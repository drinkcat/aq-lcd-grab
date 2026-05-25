mod bus_decoder;
mod decoder;
mod framebuffer;
mod permute;
mod wire;

use std::collections::HashSet;
use std::hash::{Hash, Hasher};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::{Context, bail};
use clap::{Parser, ValueEnum};
use eframe::egui;
use bus_decoder::{BusDecoder, Frame};
use framebuffer::{Framebuffer, WindowWrite};
use wire::{Decoder as WireDecoder, Event, HOST_CMD_START, HOST_CMD_STOP};

#[derive(Copy, Clone, Debug, ValueEnum)]
enum Board {
    /// Pico 2 W (USB CDC, GPIO0..15 = DB0..15, GPIO16=DC, GPIO17=CS).
    Pico,
    /// F103 capture board (UART, PA0..7=DB0..7, PB0..1/PB3..8=DB8..15,
    /// PB10=DC, PB11=CS).
    F103,
}

impl Board {
    fn permute(self, sample: u32) -> (u16, bool) {
        match self {
            Board::Pico => permute::permute_pico(sample),
            Board::F103 => permute::permute_f103(sample),
        }
    }
}

#[derive(Parser, Debug)]
#[command(about = "Live viewer for the aq-lcd-grab firmware capture stream")]
struct Args {
    /// Serial device the firmware is logging on.
    #[arg(short, long, default_value = "/dev/ttyACM0")]
    port: String,

    /// Which capture board is on the other end. Picks the permutation
    /// from raw (pa, pb) port reads back to logical (data, dc, cs).
    #[arg(short, long, value_enum, default_value_t = Board::Pico)]
    board: Board,

    /// Optional file to replay (skips opening the serial port).
    /// Raw binary frames as emitted by the firmware.
    #[arg(short, long)]
    replay: Option<String>,

    /// Directory to dump per-glyph PNGs into. One PNG is written each time
    /// a MEMORY_WRITE exactly fills its addressed window; identical
    /// (window, content) pairs are deduplicated.
    #[arg(long)]
    dump_dir: Option<PathBuf>,
}

/// ILI9488 command names.
fn cmd_name(cmd: u8) -> &'static str {
    match cmd {
        0x00 => "NOP",
        0x01 => "SOFT_RESET",
        0x04 => "READ_DISPLAY_ID",
        0x09 => "READ_DISPLAY_STATUS",
        0x10 => "SLEEP_IN",
        0x11 => "SLEEP_OUT",
        0x12 => "PARTIAL_MODE_ON",
        0x13 => "NORMAL_MODE_ON",
        0x20 => "INVERSION_OFF",
        0x21 => "INVERSION_ON",
        0x28 => "DISPLAY_OFF",
        0x29 => "DISPLAY_ON",
        0x2A => "SET_COLUMN_ADDRESS",
        0x2B => "SET_ROW_ADDRESS",
        0x2C => "MEMORY_WRITE",
        0x2E => "MEMORY_READ",
        0x30 => "PARTIAL_AREA",
        0x33 => "VERTICAL_SCROLLING",
        0x36 => "MEMORY_ACCESS_CONTROL",
        0x37 => "VERTICAL_SCROLL_START",
        0x38 => "IDLE_MODE_OFF",
        0x39 => "IDLE_MODE_ON",
        0x3A => "PIXEL_FORMAT_SET",
        0x3C => "MEMORY_WRITE_CONTINUE",
        0xB0 => "INTERFACE_MODE_CONTROL",
        0xB1 => "FRAME_RATE_NORMAL",
        0xB4 => "DISPLAY_INVERSION_CONTROL",
        0xB6 => "DISPLAY_FUNCTION_CONTROL",
        0xB7 => "ENTRY_MODE_SET",
        0xC0 => "POWER_CONTROL_1",
        0xC1 => "POWER_CONTROL_2",
        0xC5 => "VCOM_CONTROL",
        0xE0 => "POSITIVE_GAMMA",
        0xE1 => "NEGATIVE_GAMMA",
        0xF7 => "ADJUST_CONTROL_3",
        _ => "?",
    }
}

struct Shared {
    fb: Framebuffer,
    log: std::collections::VecDeque<LogEntry>,
    /// Latest decoded value per named row (see decoder::ROWS). Updated
    /// from the reader thread; rendered in the top panel.
    values: std::collections::BTreeMap<&'static str, String>,
}

#[derive(Clone)]
enum LogEntry {
    Tx(Frame),
    Msg(String),
}

const LOG_CAP: usize = 128;

fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    // The reader thread blocks in serial I/O and would otherwise keep
    // the process alive through Ctrl-C. Hard-exit on SIGINT.
    ctrlc::set_handler(|| std::process::exit(0))?;

    let shared = Arc::new(Mutex::new(Shared {
        fb: Framebuffer::new(),
        log: std::collections::VecDeque::with_capacity(LOG_CAP),
        values: std::collections::BTreeMap::new(),
    }));

    if let Some(dir) = &args.dump_dir {
        std::fs::create_dir_all(dir)
            .with_context(|| format!("creating dump dir {}", dir.display()))?;
    }

    let reader_shared = Arc::clone(&shared);
    let port = args.port.clone();
    let replay = args.replay.clone();
    let dump_dir = args.dump_dir.clone();
    let board = args.board;
    thread::spawn(move || {
        if let Err(e) = reader_loop(port, board, replay, dump_dir, reader_shared) {
            eprintln!("reader thread exited: {e:#}");
        }
    });

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([800.0, 700.0]),
        ..Default::default()
    };
    eframe::run_native(
        "aq-lcd-grab viewer",
        native_options,
        Box::new(move |_cc| Ok(Box::new(App::new(Arc::clone(&shared))))),
    )
    .map_err(|e| anyhow::anyhow!("eframe error: {e}"))?;

    // The reader thread is parked in a blocking serial read with a 60 s
    // timeout. Returning here would wait for it (which makes Ctrl-C feel
    // unresponsive) — exit hard instead.
    std::process::exit(0);
}

fn reader_loop(
    port: String,
    board: Board,
    replay: Option<String>,
    dump_dir: Option<PathBuf>,
    shared: Arc<Mutex<Shared>>,
) -> anyhow::Result<()> {
    let (mut reader, mut writer): (Box<dyn Read + Send>, Option<Box<dyn Write + Send>>) =
        if let Some(path) = replay {
            let f = std::fs::File::open(&path).with_context(|| format!("opening {path}"))?;
            (Box::new(f), None)
        } else {
            // Baud doesn't matter for the Pico (USB CDC ignores it) but
            // gives the right rate for UART-attached boards like the
            // F103.
            // Read timeout: the main loop treats a timed-out read as
            // "no bytes" and pumps the glyph-row settler. The sync
            // handshake's drain-until-quiet also uses this as its
            // quiet window.
            let mut port_handle = serialport::new(&port, 921_600)
                .timeout(Duration::from_millis(50))
                // Non-exclusive so `printf '\x04' > /dev/ttyUSB0` can
                // poke STATS while the viewer is running. Garbled
                // bytes are possible if someone reads concurrently,
                // but writes from another process are fine.
                .exclusive(false)
                .open()
                .with_context(|| format!("opening serial port {port}"))?;
            // F103 wires DTR→BOOT0 and RTS→NRST through 1k resistors
            // (see firmware-stm32/scripts/flash-uart.sh). Without
            // intervention, Linux's tty open() leaves these in a state
            // that drops the chip into the ROM bootloader instead of
            // user code, every time the viewer launches.
            //
            // Drive a clean "reset into user code" pulse:
            //   DTR=true  → BOOT0 low (run user app)
            //   RTS=true  → NRST low  (assert reset)
            //   <20 ms>
            //   RTS=false → NRST high (release reset, chip boots)
            //
            // The exact wire polarity depends on the FT232R EEPROM /
            // adapter wiring (some clones invert, some don't); the
            // values below are right for this rig. Harmless on
            // USB-CDC boards (Pico) where DTR/RTS aren't wired to
            // anything.
            port_handle
                .write_data_terminal_ready(true)
                .with_context(|| "DTR=true (BOOT0 low → run user code)")?;
            port_handle
                .write_request_to_send(true)
                .with_context(|| "RTS=true (NRST low → reset asserted)")?;
            std::thread::sleep(Duration::from_millis(20));
            port_handle
                .write_request_to_send(false)
                .with_context(|| "RTS=false (NRST released)")?;
            // Give the F103 a moment to boot its firmware before we
            // start the STOP/START handshake.
            std::thread::sleep(Duration::from_millis(250));
            let writer = port_handle
                .try_clone()
                .with_context(|| "cloning serial handle for writer")?;
            eprintln!("reader: opened {port}");
            (Box::new(port_handle), Some(Box::new(writer)))
        };

    if let Some(w) = writer.as_mut() {
        sync(reader.as_mut(), w.as_mut())?;
    }

    let mut wire = WireDecoder::new();
    let mut bus = BusDecoder::new();
    let mut glyphs = decoder::Decoder::new();
    let mut seen: HashSet<u64> = HashSet::new();
    let mut buf = [0u8; 4096];

    loop {
        let read = match reader.read(&mut buf) {
            Ok(0) => bail!("stream EOF"),
            Ok(n) => n,
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => 0,
            Err(e) => return Err(e.into()),
        };
        let events = if read > 0 {
            wire.feed(&buf[..read])?
        } else {
            Vec::new()
        };

        let mut g = shared.lock().unwrap();
        for ev in events {
            dispatch_event(ev, &mut g, &mut bus, &mut glyphs, dump_dir.as_deref(), &mut seen, board);
        }
        if read == 0 {
            // Idle pump: settle any glyph rows that have stopped updating.
            for r in glyphs.flush() {
                let msg = format!("= {}: {:?}", r.name, r.value);
                println!("{msg}");
                push_log(&mut g.log, LogEntry::Msg(msg.clone()));
                g.values.insert(r.name, r.value);
            }
        }
    }
}

/// Process one wire event: permute samples, feed the bus decoder,
/// hand any completed 8080 transactions to `handle_frame`.
fn dispatch_event(
    ev: Event,
    g: &mut Shared,
    bus: &mut BusDecoder,
    glyphs: &mut decoder::Decoder,
    dump_dir: Option<&Path>,
    seen: &mut HashSet<u64>,
    board: Board,
) {
    match ev {
        Event::Block(samples) => {
            print!("BLOCK n={:3}", samples.len());
            for s in &samples {
                let (data, is_data) = board.permute(*s);
                // 'D' = data word, 'C' = command byte.
                print!(" {}:{:04x}", if is_data { 'D' } else { 'C' }, data);
            }
            println!();
            for s in samples {
                let (data, is_data) = board.permute(s);
                if let Some(tx) = bus.feed(data, is_data) {
                    handle_frame(g, glyphs, dump_dir, seen, tx);
                }
            }
        }
        Event::Run { n, sample } => {
            let (data, is_data) = board.permute(sample);
            println!(
                "RUN   n={:3} {}:{:04x}",
                n,
                if is_data { 'D' } else { 'C' },
                data
            );
            if let Some(tx) = bus.feed_run(n as usize, data, is_data) {
                handle_frame(g, glyphs, dump_dir, seen, tx);
            }
        }
        Event::Overrun { dropped } => {
            let msg = format!("(firmware lost {dropped} WR edges)");
            println!("! {msg}");
            push_log(&mut g.log, LogEntry::Msg(msg));
        }
        Event::Log(text) => {
            println!("• {text}");
            push_log(&mut g.log, LogEntry::Msg(text));
        }
        Event::Started => push_log(&mut g.log, LogEntry::Msg("STARTED".into())),
        Event::Stopped => push_log(&mut g.log, LogEntry::Msg("STOPPED".into())),
    }
}

/// Replays `frame` into the framebuffer + glyph decoder, dumps the
/// resulting window if a dump dir is set, and pushes log lines.
fn handle_frame(
    g: &mut Shared,
    glyphs: &mut decoder::Decoder,
    dump_dir: Option<&Path>,
    seen: &mut HashSet<u64>,
    frame: Frame,
) {
    println!("{}", format_tx(&frame));
    let win = g.fb.apply(&frame);
    push_log(&mut g.log, LogEntry::Tx(frame));
    if let Some(win) = win.as_ref() {
        let out = glyphs.ingest(win);
        if let Some(m) = out.glyph {
            let msg = format!(
                "→ {}x{}@({:03},{:03}) = {:?}",
                m.w, m.h, m.disp_x, m.disp_y, m.label
            );
            println!("{msg}");
            push_log(&mut g.log, LogEntry::Msg(msg));
        }
        for r in out.completed_rows {
            let msg = format!("= {}: {:?}", r.name, r.value);
            println!("{msg}");
            push_log(&mut g.log, LogEntry::Msg(msg.clone()));
            g.values.insert(r.name, r.value);
        }
    }
    if let (Some(dir), Some(win)) = (dump_dir, win) {
        dump_window(dir, &win, seen);
    }
}

/// Handshake: drain any backlog, send STOP and wait for `0xFC`, then
/// send START and wait for `0xFB`. After this returns, the next byte
/// read starts a fresh frame.
/// Handshake. Send STOP, drain until the firmware stops talking,
/// sanity-check that the last byte we saw was the STOPPED ack
/// (0xFC). Then send START — the first byte from the firmware should
/// be the STARTED ack (0xFB), and everything after it is fresh frame
/// data going to the wire decoder.
fn sync(
    reader: &mut (dyn Read + Send),
    writer: &mut (dyn Write + Send),
) -> anyhow::Result<()> {
    // 1. Tell the firmware to stop sending. Then read until quiet —
    //    anything in the OS buffer plus anything the firmware was
    //    mid-emitting plus the STOPPED ack will all flush. "Quiet" =
    //    one serial-port read timeout (250 ms) without any bytes.
    //    After it goes quiet, the last byte we saw must be 0xFC;
    //    otherwise the firmware isn't speaking our protocol.
    writer.write_all(&[HOST_CMD_STOP])?;
    writer.flush()?;
    if !drain_until_quiet(reader, 0xFC)? {
        bail!("never saw STOPPED ack (0xFC) after sending STOP");
    }

    // 2. Send START. Everything from here on — including the FB
    //    STARTED ack — flows to the main loop's wire decoder. No
    //    explicit ack check; the wire decoder will surface FB as
    //    Event::Started and we'll see it in the activity log.
    writer.write_all(&[HOST_CMD_START])?;
    writer.flush()?;
    eprintln!("reader: synced (sent START)");
    Ok(())
}

/// Read from `reader` until it times out (no bytes arrived for one
/// read-timeout window). Returns whether `needle` appeared anywhere
/// in the drained bytes.
fn drain_until_quiet(
    reader: &mut (dyn Read + Send),
    needle: u8,
) -> anyhow::Result<bool> {
    let mut saw_needle = false;
    let mut buf = [0u8; 4096];
    loop {
        match reader.read(&mut buf) {
            Ok(0) => return Ok(saw_needle),
            Ok(n) => {
                if buf[..n].contains(&needle) {
                    saw_needle = true;
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => return Ok(saw_needle),
            Err(e) => return Err(e.into()),
        }
    }
}


/// Minimum window dimension considered a glyph worth keeping. Filters
/// out thin status bars and 1-pixel-wide animation slivers.
const GLYPH_MIN_DIM: u16 = 8;

/// Save a per-window PNG into `dir`. Each pixel is reduced to a 1-bit
/// foreground/background mask (majority pixel value = background), so the
/// same digit collapses to one file across red/green/white backgrounds.
/// Dedup key is (size, mask) — position is recorded in the filename only.
fn dump_window(dir: &Path, win: &WindowWrite, seen: &mut HashSet<u64>) {
    if win.w < GLYPH_MIN_DIM || win.h < GLYPH_MIN_DIM {
        return;
    }

    let bg = win.pixels[0];
    let mask: Vec<bool> = win.pixels.iter().map(|&p| p != bg).collect();

    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    win.w.hash(&mut hasher);
    win.h.hash(&mut hasher);
    for &m in &mask {
        m.hash(&mut hasher);
    }
    let key = hasher.finish();
    if !seen.insert(key) {
        return;
    }

    let disp_x = framebuffer::WIDTH.saturating_sub(win.x + win.w);
    let disp_y = framebuffer::HEIGHT.saturating_sub(win.y + win.h);

    let mut rgba = Vec::with_capacity(mask.len() * 4);
    for &m in mask.iter().rev() {
        let v = if m { 0xFF } else { 0 };
        rgba.push(v);
        rgba.push(v);
        rgba.push(v);
        rgba.push(0xFF);
    }
    let ts = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis())
        .unwrap_or(0);
    let name = format!(
        "{}x{}_{:03}_{:03}_{}.png",
        win.w, win.h, disp_x, disp_y, ts
    );
    let path = dir.join(&name);
    match image::RgbaImage::from_raw(win.w as u32, win.h as u32, rgba) {
        Some(img) => {
            if let Err(e) = img.save(&path) {
                eprintln!("dumper: save {} failed: {e}", path.display());
            } else {
                println!("• dumped {name}");
            }
        }
        None => eprintln!("dumper: rgba buffer size mismatch"),
    }
}

fn format_tx(tx: &Frame) -> String {
    let mut line = format!("{:#04x} {}", tx.cmd, cmd_name(tx.cmd));
    if !tx.data.is_empty() {
        line.push_str(&format!(" [{}]", tx.data.len()));
        for (i, w) in tx.data.iter().take(6).enumerate() {
            if i == 0 {
                line.push(' ');
            }
            line.push_str(&format!("{w:04x} "));
        }
        if tx.data.len() > 6 {
            line.push('…');
        }
    }
    line
}

fn push_log(log: &mut std::collections::VecDeque<LogEntry>, entry: LogEntry) {
    if log.len() == LOG_CAP {
        log.pop_front();
    }
    log.push_back(entry);
}

struct App {
    shared: Arc<Mutex<Shared>>,
    texture: Option<egui::TextureHandle>,
}

impl App {
    fn new(shared: Arc<Mutex<Shared>>) -> Self {
        Self {
            shared,
            texture: None,
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();

        let (rgba, w, h, log, values) = {
            let g = self.shared.lock().unwrap();
            (
                g.fb.to_rgba8(),
                framebuffer::WIDTH as usize,
                framebuffer::HEIGHT as usize,
                g.log.iter().cloned().collect::<Vec<_>>(),
                g.values.clone(),
            )
        };

        let image = egui::ColorImage::from_rgba_unmultiplied([w, h], &rgba);
        match &mut self.texture {
            Some(t) => t.set(image, egui::TextureOptions::NEAREST),
            None => {
                self.texture = Some(ctx.load_texture(
                    "framebuffer",
                    image,
                    egui::TextureOptions::NEAREST,
                ));
            }
        }

        egui::Panel::top("values").show_inside(ui, |ui| {
            ui.add_space(4.0);
            ui.horizontal_wrapped(|ui| {
                if values.is_empty() {
                    ui.weak("(no values decoded yet)");
                }
                for (name, v) in &values {
                    ui.group(|ui| {
                        ui.vertical(|ui| {
                            ui.weak(*name);
                            ui.label(
                                egui::RichText::new(v)
                                    .strong()
                                    .size(22.0)
                                    .monospace(),
                            );
                        });
                    });
                }
            });
            ui.add_space(4.0);
        });

        egui::Panel::right("log")
            .resizable(true)
            .default_size(300.0)
            .show_inside(ui, |ui| {
                ui.heading("Recent activity");
                egui::ScrollArea::vertical().show(ui, |ui| {
                    for entry in log.iter().rev() {
                        match entry {
                            LogEntry::Msg(s) => {
                                ui.colored_label(egui::Color32::LIGHT_BLUE, format!("• {s}"));
                            }
                            LogEntry::Tx(tx) => {
                                ui.label(format_tx(tx));
                            }
                        }
                    }
                });
            });

        egui::CentralPanel::default().show_inside(ui, |ui| {
            if let Some(t) = &self.texture {
                let avail = ui.available_size();
                let aspect = w as f32 / h as f32;
                let target_w = (avail.y * aspect).min(avail.x);
                let target_h = target_w / aspect;
                ui.image((t.id(), egui::vec2(target_w, target_h)));
            }
        });

        ctx.request_repaint_after(Duration::from_millis(33));
    }
}
