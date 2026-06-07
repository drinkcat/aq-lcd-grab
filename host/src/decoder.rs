//! Match captured glyph windows against a baked-in template table and
//! assemble per-row values once each row settles.
//!
//! The decoder operates directly on the permuted `(data, is_data)` bus
//! stream — the same level as the wire encoder on the firmware side —
//! without depending on `BusDecoder` or `Framebuffer`. It does its own
//! minimal 8080 framing, accumulates pixels as an RLE hash, and matches
//! completed windows against pre-hashed templates.
//!
//! Templates are generated at build time from `templates/<W>x<H>/<label>.png`
//! by `build.rs`. Each template stores a hash of its binarized pixel stream
//! (RLE over fg/bg runs, same convention as the runtime accumulator).
//!
//! Glyphs are routed to fixed metric rows declared in [`ROWS`]. Digits are
//! stored left-to-right by display-space x. The caller invokes [`Decoder::flush`]
//! when it detects idleness (serial read timeout / drain-loop idle) to emit
//! any dirty rows.

use crate::framebuffer;
use crate::fnv::{fnv_init, fnv_mix};

include!(concat!(env!("OUT_DIR"), "/templates_gen.rs"));

/// One metric region on the display. A glyph belongs to this row if its
/// display-space (x, y) top-left lands inside `[x_min, x_max) × [y_min, y_max)`.
struct RowDef {
    name: &'static str,
    x_min: u16,
    x_max: u16,
    y_min: u16,
    y_max: u16,
}

static ROWS: &[RowDef] = &[
    // Top large red panel — PM2.5 (μg/m³ unit label below).
    RowDef { name: "pm25",     x_min:  80, x_max: 230, y_min:  50, y_max: 130 },
    // Second large panel — TVOC (2-digit + decimal dot in the middle).
    RowDef { name: "tvoc",     x_min:  80, x_max: 230, y_min: 195, y_max: 280 },
    // Mid green panel — CO2 (4-digit ppm).
    RowDef { name: "co2",      x_min:  90, x_max: 245, y_min: 350, y_max: 405 },
    // Bottom-left — temperature.
    RowDef { name: "temp",     x_min:   0, x_max: 100, y_min: 425, y_max: 475 },
    // Bottom-right — humidity.
    RowDef { name: "humidity", x_min: 200, x_max: 320, y_min: 425, y_max: 475 },
];

const ROWS_LEN: usize = 5;
const MAX_DIGITS_PER_ROW: usize = 8;

// ---- Internal state ----

/// In-progress glyph window accumulation.
struct PendingWindow {
    x: u16,
    y: u16,
    w: u16,
    h: u16,
    bg: u16,
    /// FNV-1a accumulator over the RLE run-length sequence.
    hash: u64,
    /// Length of the current bg/fg run.
    run_len: u16,
    /// Whether the current run is foreground.
    run_is_fg: bool,
    /// Pixels received so far.
    pixel_count: u32,
}

impl PendingWindow {
    fn new(x: u16, y: u16, w: u16, h: u16) -> Self {
        Self {
            x, y, w, h,
            bg: 0,
            hash: fnv_init(),
            run_len: 0,
            run_is_fg: false,
            pixel_count: 0,
        }
    }

    /// Feed one pixel. Returns the completed hash when the window is full.
    fn push(&mut self, pixel: u16) -> Option<u64> {
        if self.pixel_count == 0 {
            self.bg = pixel;
            self.run_len = 1;
            self.run_is_fg = false;
        } else {
            let is_fg = pixel != self.bg;
            if is_fg == self.run_is_fg {
                self.run_len += 1;
            } else {
                // Run boundary: mix the completed run length into the hash.
                // Mix two bytes (little-endian u16) so longer runs don't
                // alias shorter ones at a byte boundary.
                self.hash = fnv_mix(self.hash, self.run_len as u8);
                self.hash = fnv_mix(self.hash, (self.run_len >> 8) as u8);
                self.run_len = 1;
                self.run_is_fg = is_fg;
            }
        }
        self.pixel_count += 1;
        let total = self.w as u32 * self.h as u32;
        if self.pixel_count == total {
            // Mix the final run.
            let mut h = fnv_mix(self.hash, self.run_len as u8);
            h = fnv_mix(h, (self.run_len >> 8) as u8);
            Some(h)
        } else {
            None
        }
    }
}

#[derive(Default)]
struct RowState {
    digits: [Option<(u16, &'static str)>; MAX_DIGITS_PER_ROW],
    n_digits: usize,
    dirty: bool,
}

impl RowState {
    fn insert(&mut self, disp_x: u16, label: &'static str) {
        // Find insertion point to keep digits sorted by x.
        let pos = self.digits[..self.n_digits]
            .iter()
            .position(|s| s.unwrap().0 >= disp_x);
        match pos {
            Some(i) if self.digits[i].unwrap().0 == disp_x => {
                self.digits[i] = Some((disp_x, label));
            }
            Some(i) => {
                if self.n_digits < MAX_DIGITS_PER_ROW {
                    self.digits[i..self.n_digits + 1].rotate_right(1);
                    self.digits[i] = Some((disp_x, label));
                    self.n_digits += 1;
                }
            }
            None => {
                if self.n_digits < MAX_DIGITS_PER_ROW {
                    self.digits[self.n_digits] = Some((disp_x, label));
                    self.n_digits += 1;
                }
            }
        }
        self.dirty = true;
    }
}

// ---- 8080 command framing state ----

#[derive(Default)]
enum Cmd {
    #[default]
    None,
    /// Collecting 4 address bytes for SET_COLUMN_ADDRESS (0x2A).
    ColAddr { buf: [u8; 4], n: u8 },
    /// Collecting 4 address bytes for SET_ROW_ADDRESS (0x2B).
    RowAddr { buf: [u8; 4], n: u8 },
    /// Receiving pixel data for MEMORY_WRITE (0x2C / 0x3C).
    MemWrite,
}

// ---- Public types ----

pub struct GlyphMatch {
    pub disp_x: u16,
    pub disp_y: u16,
    pub w: u16,
    pub h: u16,
    pub label: &'static str,
}

pub struct RowReport {
    pub name: &'static str,
    pub value: String,
}

pub struct DecodeOut {
    pub glyph: Option<GlyphMatch>,
}

// ---- Decoder ----

#[derive(Default)]
pub struct Decoder {
    col_start: u16,
    col_end: u16,
    row_start: u16,
    row_end: u16,
    cmd: Cmd,
    pending: Option<PendingWindow>,
    rows: [RowState; ROWS_LEN],
}

impl Decoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed one permuted bus sample `(data, is_data)`.
    /// Returns `Some(DecodeOut)` when a glyph window completes.
    pub fn feed(&mut self, data: u16, is_data: bool) -> Option<DecodeOut> {
        if !is_data {
            // New command byte: finalise any in-progress command first.
            self.finalise_cmd();
            self.cmd = match data as u8 {
                0x2A => Cmd::ColAddr { buf: [0; 4], n: 0 },
                0x2B => Cmd::RowAddr { buf: [0; 4], n: 0 },
                0x2C => {
                    self.pending = self.open_window();
                    Cmd::MemWrite
                }
                0x3C => {
                    // MEMORY_WRITE_CONTINUE: keep existing pending window.
                    Cmd::MemWrite
                }
                _ => Cmd::None,
            };
            return None;
        }

        match &mut self.cmd {
            Cmd::ColAddr { buf, n } => {
                if (*n as usize) < 4 {
                    buf[*n as usize] = (data & 0xFF) as u8;
                    *n += 1;
                    if *n == 4 {
                        self.col_start = (buf[0] as u16) << 8 | buf[1] as u16;
                        self.col_end   = (buf[2] as u16) << 8 | buf[3] as u16;
                    }
                }
                None
            }
            Cmd::RowAddr { buf, n } => {
                if (*n as usize) < 4 {
                    buf[*n as usize] = (data & 0xFF) as u8;
                    *n += 1;
                    if *n == 4 {
                        self.row_start = (buf[0] as u16) << 8 | buf[1] as u16;
                        self.row_end   = (buf[2] as u16) << 8 | buf[3] as u16;
                    }
                }
                None
            }
            Cmd::MemWrite => {
                if let Some(win) = self.pending.as_mut() {
                    if let Some(hash) = win.push(data) {
                        let glyph = self.finish_window(hash);
                        self.pending = None;
                        return Some(DecodeOut { glyph });
                    }
                }
                None
            }
            Cmd::None => None,
        }
    }

    /// Called by the main loop when idle. Emits RowReports for dirty rows.
    pub fn flush(&mut self) -> Vec<RowReport> {
        let mut out = Vec::new();
        for (def, state) in ROWS.iter().zip(self.rows.iter_mut()) {
            if !state.dirty {
                continue;
            }
            let value: String = state.digits[..state.n_digits]
                .iter()
                .map(|s| short_label(s.unwrap().1))
                .collect();
            out.push(RowReport { name: def.name, value });
            state.dirty = false;
        }
        out
    }

    /// Open a PendingWindow if the current col/row window intersects a
    /// known metric region and is a plausible glyph size.
    fn open_window(&self) -> Option<PendingWindow> {
        if self.col_end < self.col_start || self.row_end < self.row_start {
            return None;
        }
        let w = self.col_end - self.col_start + 1;
        let h = self.row_end - self.row_start + 1;
        // Only open for sizes that exist in the template table.
        if !TEMPLATES.iter().any(|t| t.w == w && t.h == h) {
            return None;
        }
        let disp_x = framebuffer::WIDTH.saturating_sub(self.col_start + w);
        let disp_y = framebuffer::HEIGHT.saturating_sub(self.row_start + h);
        if row_for(disp_x, disp_y).is_none() {
            return None;
        }
        Some(PendingWindow::new(self.col_start, self.row_start, w, h))
    }

    /// A window just completed with the given hash. Match against templates
    /// and record the glyph in the appropriate row.
    fn finish_window(&mut self, hash: u64) -> Option<GlyphMatch> {
        let win = self.pending.as_ref()?;
        let w = win.w;
        let h = win.h;
        let disp_x = framebuffer::WIDTH.saturating_sub(win.x + w);
        let disp_y = framebuffer::HEIGHT.saturating_sub(win.y + h);

        let label = TEMPLATES
            .iter()
            .find(|t| t.w == w && t.h == h && t.hash == hash)
            .map(|t| t.label)
            .unwrap_or("#");

        if let Some(idx) = row_for(disp_x, disp_y) {
            self.rows[idx].insert(disp_x, label);
        }

        Some(GlyphMatch { disp_x, disp_y, w, h, label })
    }

    fn finalise_cmd(&mut self) {
        // ColAddr/RowAddr state is committed incrementally as bytes arrive;
        // nothing extra to do. MemWrite pending windows that didn't fill
        // (partial write) are simply dropped.
        if matches!(self.cmd, Cmd::MemWrite) {
            self.pending = None;
        }
    }
}

fn row_for(x: u16, y: u16) -> Option<usize> {
    ROWS.iter()
        .position(|r| x >= r.x_min && x < r.x_max && y >= r.y_min && y < r.y_max)
}

fn short_label(label: &str) -> char {
    match label {
        "blank" => ' ',
        "dot"   => '.',
        other   => other.chars().next().unwrap_or('?'),
    }
}
