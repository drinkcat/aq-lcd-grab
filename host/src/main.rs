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
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, anyhow, bail};
use clap::Parser;
use eframe::egui;
use bus_decoder::{BusDecoder, Frame};
use framebuffer::{Framebuffer, WindowWrite};
use wire::{Decoder as WireDecoder, Event, HOST_CMD_START, HOST_CMD_STOP};

#[derive(Parser, Debug)]
#[command(about = "Live viewer for the aq-lcd-grab firmware capture stream")]
struct Args {
    /// Serial device the firmware is logging on.
    #[arg(short, long, default_value = "/dev/ttyACM0")]
    port: String,

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
    thread::spawn(move || {
        if let Err(e) = reader_loop(port, replay, dump_dir, reader_shared) {
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
            let port_handle = serialport::new(&port, 921_600)
                .timeout(Duration::from_millis(250))
                .open()
                .with_context(|| format!("opening serial port {port}"))?;
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
            match ev {
                Event::Block(samples) => {
                    for (pa, pb) in samples {
                        let (data, dc, _cs) = permute::permute_pico(pa, pb);
                        if let Some(tx) = bus.feed(data, dc) {
                            handle_frame(&mut g, &mut glyphs, dump_dir.as_deref(), &mut seen, tx);
                        }
                    }
                }
                Event::Run { n, pa, pb } => {
                    let (data, dc, _cs) = permute::permute_pico(pa, pb);
                    if let Some(tx) = bus.feed_run(n as usize, data, dc) {
                        handle_frame(&mut g, &mut glyphs, dump_dir.as_deref(), &mut seen, tx);
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
                Event::Started | Event::Stopped => {
                    // Steady-state acks: just note them in the log.
                    let s = if matches!(ev, Event::Started) { "STARTED" } else { "STOPPED" };
                    push_log(&mut g.log, LogEntry::Msg(s.into()));
                }
            }
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
fn sync(reader: &mut (dyn Read + Send), writer: &mut (dyn Write + Send)) -> anyhow::Result<()> {
    let mut scratch = [0u8; 4096];

    // 1. Drain everything currently buffered (250 ms is much longer
    //    than any in-flight frame at 921600 baud).
    let drain_deadline = Instant::now() + Duration::from_millis(250);
    while Instant::now() < drain_deadline {
        match reader.read(&mut scratch) {
            Ok(0) | Err(_) => break,
            Ok(_) => {}
        }
    }

    // 2. Send STOP, scan for 0xFC.
    writer.write_all(&[HOST_CMD_STOP])?;
    writer.flush()?;
    scan_for(reader, 0xFC, "STOPPED ack")?;

    // 3. Send START, scan for 0xFB.
    writer.write_all(&[HOST_CMD_START])?;
    writer.flush()?;
    scan_for(reader, 0xFB, "STARTED ack")?;

    eprintln!("reader: synced (STARTED)");
    Ok(())
}

/// Read bytes until `target` shows up. Discards anything before it.
fn scan_for(reader: &mut (dyn Read + Send), target: u8, what: &str) -> anyhow::Result<()> {
    let deadline = Instant::now() + Duration::from_secs(2);
    let mut buf = [0u8; 256];
    while Instant::now() < deadline {
        match reader.read(&mut buf) {
            Ok(0) => return Err(anyhow!("EOF while waiting for {what}")),
            Ok(n) => {
                if buf[..n].contains(&target) {
                    return Ok(());
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(e) => return Err(e.into()),
        }
    }
    Err(anyhow!("timeout waiting for {what}"))
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
