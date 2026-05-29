//! Match captured glyph windows against a baked-in template table and
//! assemble per-row values once each row settles.
//!
//! Templates are generated at build time from `templates/<W>x<H>/<label>.png`
//! by `build.rs` and included as a packed-bit array. Matching is an exact
//! mask compare: binarize the window (pixel at 0,0 = bg, everything else
//! = fg), pack to 1 bit per pixel, then look for the template with the
//! same (W, H, mask). O(N) over templates, O(W·H/8) per compare — cheap
//! enough to run inline in the capture path on either host or Pico.
//!
//! Glyphs are routed to fixed metric rows declared in [`ROWS`]: each row
//! covers an (x, y) display-space rectangle that the panel uses for one
//! metric. Digits are stored left-to-right by x, and the row is emitted
//! once it has been idle for `FLUSH_QUIET_MS`. Pumping `flush()` from the
//! read loop is sufficient since frames arrive at least once per second.

use std::time::{Duration, Instant};

use crate::framebuffer::{self, WindowWrite};

include!(concat!(env!("OUT_DIR"), "/templates_gen.rs"));

const FLUSH_QUIET_MS: u64 = 500;

/// One metric region on the display. A glyph belongs to this row if its
/// display-space (x, y) top-left lands inside the half-open rectangle
/// `[x_min, x_max) × [y_min, y_max)`.
struct RowDef {
    name: &'static str,
    x_min: u16,
    x_max: u16,
    y_min: u16,
    y_max: u16,
}

static ROWS: &[RowDef] = &[
    // Top large red panel — PM2.5 (μg/m³ unit label below).
    RowDef {
        name: "pm25",
        x_min: 80,
        x_max: 230,
        y_min: 50,
        y_max: 130,
    },
    // Second large panel — TVOC (2-digit + decimal dot in the middle).
    RowDef {
        name: "tvoc",
        x_min: 80,
        x_max: 230,
        y_min: 195,
        y_max: 280,
    },
    // Mid green panel — CO2 (4-digit ppm).
    RowDef {
        name: "co2",
        x_min: 90,
        x_max: 245,
        y_min: 350,
        y_max: 405,
    },
    // Bottom-left — temperature.
    RowDef {
        name: "temp",
        x_min: 0,
        x_max: 100,
        y_min: 425,
        y_max: 475,
    },
    // Bottom-right — humidity.
    RowDef {
        name: "humidity",
        x_min: 200,
        x_max: 320,
        y_min: 425,
        y_max: 475,
    },
];

#[derive(Default)]
pub struct Decoder {
    rows: [RowState; ROWS_LEN],
}

const ROWS_LEN: usize = 5;

#[derive(Default)]
struct RowState {
    /// Digits keyed by display-space x; iterated left-to-right.
    digits: Vec<(u16, &'static str)>,
    last_update: Option<Instant>,
    dirty: bool,
}

/// Result of feeding a window: 0 or 1 single-glyph matches plus any rows
/// that have just gone quiet and should be emitted to the log.
pub struct DecodeOut {
    pub glyph: Option<GlyphMatch>,
    pub completed_rows: Vec<RowReport>,
}

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

impl Decoder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn ingest(&mut self, win: &WindowWrite) -> DecodeOut {
        let mut out = DecodeOut {
            glyph: None,
            completed_rows: Vec::new(),
        };
        // Only record windows whose size matches a known template family.
        // Unmatched glyphs at those sizes become "#" placeholders so the
        // value string keeps its column alignment.
        if TEMPLATES.iter().any(|t| t.w == win.w && t.h == win.h) {
            let label = match_glyph(win).unwrap_or("#");
            let disp_x = framebuffer::WIDTH.saturating_sub(win.x + win.w);
            let disp_y = framebuffer::HEIGHT.saturating_sub(win.y + win.h);
            if let Some(idx) = row_for(disp_x, disp_y) {
                let row = &mut self.rows[idx];
                match row.digits.binary_search_by_key(&disp_x, |(x, _)| *x) {
                    Ok(i) => row.digits[i].1 = label,
                    Err(i) => row.digits.insert(i, (disp_x, label)),
                }
                row.last_update = Some(Instant::now());
                row.dirty = true;
            }
            out.glyph = Some(GlyphMatch {
                disp_x,
                disp_y,
                w: win.w,
                h: win.h,
                label,
            });
        }
        out.completed_rows = self.flush();
        out
    }

    /// Emit rows that have been idle for at least `FLUSH_QUIET_MS`.
    /// Pump this from the read loop so settled values surface even when
    /// the only frames arriving are non-glyph commands.
    pub fn flush(&mut self) -> Vec<RowReport> {
        let now = Instant::now();
        let quiet = Duration::from_millis(FLUSH_QUIET_MS);
        let mut out = Vec::new();
        for (def, state) in ROWS.iter().zip(self.rows.iter_mut()) {
            if !state.dirty {
                continue;
            }
            let Some(last) = state.last_update else {
                continue;
            };
            if now.duration_since(last) < quiet {
                continue;
            }
            let value: String = state.digits.iter().map(|(_, l)| short_label(l)).collect();
            out.push(RowReport {
                name: def.name,
                value,
            });
            state.dirty = false;
        }
        out
    }
}

fn row_for(x: u16, y: u16) -> Option<usize> {
    ROWS.iter()
        .position(|r| x >= r.x_min && x < r.x_max && y >= r.y_min && y < r.y_max)
}

/// Compress a template label into a single character for value assembly.
/// Digit labels collapse to their character; named labels map to glyphs.
fn short_label(label: &str) -> char {
    match label {
        "blank" => ' ',
        "dot" => '.',
        other => other.chars().next().unwrap_or('?'),
    }
}

fn match_glyph(win: &WindowWrite) -> Option<&'static str> {
    if win.pixels.is_empty() {
        return None;
    }
    let bg = win.pixels[0];
    let n = win.pixels.len();
    // Templates are stored in display orientation (the dumper rotates the
    // raw window 180° before saving), so iterate the live pixels in
    // reverse to put the mask in the same frame of reference.
    let mut packed = vec![0u8; n.div_ceil(8)];
    for (i, &p) in win.pixels.iter().rev().enumerate() {
        if p != bg {
            packed[i / 8] |= 1 << (i % 8);
        }
    }
    for t in TEMPLATES {
        if t.w == win.w && t.h == win.h && t.mask == packed.as_slice() {
            return Some(t.label);
        }
    }
    None
}
