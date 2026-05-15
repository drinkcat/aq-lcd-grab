mod decoder;
mod framebuffer;
mod parser;
mod sample;

use std::io::{BufRead, BufReader};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use anyhow::Context;
use clap::Parser;
use decoder::{Decoder, Transaction, cmd_name};
use eframe::egui;
use framebuffer::Framebuffer;

#[derive(Parser, Debug)]
#[command(about = "Live viewer for the aq-lcd-grab firmware capture stream")]
struct Args {
    /// Serial device the firmware is logging on.
    #[arg(short, long, default_value = "/dev/ttyACM0")]
    port: String,

    /// Optional file to replay (skips opening the serial port).
    /// One firmware log line per text line.
    #[arg(short, long)]
    replay: Option<String>,
}

/// Shared state between the reader thread and the UI.
struct Shared {
    fb: Framebuffer,
    /// Most recent transactions, newest last. Capped.
    log: std::collections::VecDeque<Transaction>,
}

const LOG_CAP: usize = 64;

fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    let shared = Arc::new(Mutex::new(Shared {
        fb: Framebuffer::new(),
        log: std::collections::VecDeque::with_capacity(LOG_CAP),
    }));

    // Start the reader thread.
    let reader_shared = Arc::clone(&shared);
    let port = args.port.clone();
    let replay = args.replay.clone();
    thread::spawn(move || {
        if let Err(e) = reader_loop(port, replay, reader_shared) {
            eprintln!("reader thread exited: {e:#}");
        }
    });

    // Launch egui app.
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

    Ok(())
}

fn reader_loop(
    port: String,
    replay: Option<String>,
    shared: Arc<Mutex<Shared>>,
) -> anyhow::Result<()> {
    let mut decoder = Decoder::default();

    let reader: Box<dyn BufRead + Send> = if let Some(path) = replay {
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

    for line in reader.lines() {
        let line = match line {
            Ok(l) => l,
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(e) => return Err(e.into()),
        };
        let Some(samples) = parser::parse_line(&line) else {
            continue;
        };
        for s in samples {
            if let Some(tx) = decoder.feed(s) {
                let mut g = shared.lock().unwrap();
                g.fb.apply(&tx);
                if g.log.len() == LOG_CAP {
                    g.log.pop_front();
                }
                g.log.push_back(tx);
            }
        }
    }
    Ok(())
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

        // Pull a snapshot of the framebuffer + log.
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
            .default_size(280.0)
            .show_inside(ui, |ui| {
                ui.heading("Recent transactions");
                egui::ScrollArea::vertical().show(ui, |ui| {
                    for tx in log.iter().rev() {
                        let mut line = format!("{:#04x} {}", tx.cmd, cmd_name(tx.cmd));
                        if !tx.data.is_empty() {
                            line.push_str(&format!(" [{}]", tx.data.len()));
                            for (i, w) in tx.data.iter().take(8).enumerate() {
                                if i == 0 {
                                    line.push(' ');
                                }
                                line.push_str(&format!("{:04x} ", w));
                            }
                            if tx.data.len() > 8 {
                                line.push('…');
                            }
                        }
                        ui.label(line);
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

        // Repaint at ~30 Hz so live updates are visible.
        ctx.request_repaint_after(Duration::from_millis(33));
    }
}
