mod framebuffer;
mod wire;

use std::io::{BufReader, Read};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use anyhow::Context;
use clap::Parser;
use eframe::egui;
use framebuffer::Framebuffer;
use wire::{CMD_LOG, Frame, log_text, read_frame};

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
        0xFF => "LOG",
        _ => "?",
    }
}

struct Shared {
    fb: Framebuffer,
    log: std::collections::VecDeque<LogEntry>,
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
    }));

    let reader_shared = Arc::clone(&shared);
    let port = args.port.clone();
    let replay = args.replay.clone();
    thread::spawn(move || {
        if let Err(e) = reader_loop(port, replay, reader_shared) {
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
    shared: Arc<Mutex<Shared>>,
) -> anyhow::Result<()> {
    let mut reader: Box<dyn Read + Send> = if let Some(path) = replay {
        let f = std::fs::File::open(&path).with_context(|| format!("opening {path}"))?;
        Box::new(BufReader::new(f))
    } else {
        let port_handle = serialport::new(&port, 115_200)
            .timeout(Duration::from_secs(60))
            .open()
            .with_context(|| format!("opening serial port {port}"))?;
        eprintln!("reader: opened {port}");
        Box::new(BufReader::new(port_handle))
    };

    loop {
        let frame = match read_frame(&mut reader) {
            Ok(f) => f,
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(e) => return Err(e.into()),
        };
        let mut g = shared.lock().unwrap();
        if frame.cmd == CMD_LOG {
            let text = log_text(&frame.data);
            println!("• {text}");
            push_log(&mut g.log, LogEntry::Msg(text));
        } else {
            println!("{}", format_tx(&frame));
            g.fb.apply(&frame);
            push_log(&mut g.log, LogEntry::Tx(frame));
        }
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

        let (rgba, w, h, log) = {
            let g = self.shared.lock().unwrap();
            (
                g.fb.to_rgba8(),
                framebuffer::WIDTH as usize,
                framebuffer::HEIGHT as usize,
                g.log.iter().cloned().collect::<Vec<_>>(),
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
