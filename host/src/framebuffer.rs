//! Reconstruct the panel's framebuffer by replaying the bus transactions.
//!
//! The ILI9488/ST7796 model:
//! - `SET_COLUMN_ADDRESS (0x2A)` takes 4 bytes: start_h, start_l, end_h, end_l.
//! - `SET_ROW_ADDRESS (0x2B)` takes 4 bytes similarly.
//! - `MEMORY_WRITE (0x2C)` followed by RGB565 pixel words; the panel
//!   auto-increments column then row within the window set above.
//!
//! On a 16-bit 8080 bus, each address parameter byte is in the low 8
//! bits of a 16-bit bus word, so a SET_COLUMN_ADDRESS transaction is
//! 4 *words*, not 4 *bytes*. MEMORY_WRITE pixels are 16-bit RGB565.

use crate::bus_decoder::Frame;

pub const WIDTH: u16 = 320;
pub const HEIGHT: u16 = 480;

pub struct Framebuffer {
    pub pixels: Vec<u16>,
    pub col_start: u16,
    pub col_end: u16,
    pub row_start: u16,
    pub row_end: u16,
    pub cursor_col: u16,
    pub cursor_row: u16,
}

/// A single MEMORY_WRITE that exactly fills the active window — a candidate
/// glyph/sprite update worth dumping for offline analysis.
pub struct WindowWrite {
    pub x: u16,
    pub y: u16,
    pub w: u16,
    pub h: u16,
    pub pixels: Vec<u16>,
}

impl Framebuffer {
    pub fn new() -> Self {
        Self {
            pixels: vec![0; WIDTH as usize * HEIGHT as usize],
            col_start: 0,
            col_end: WIDTH - 1,
            row_start: 0,
            row_end: HEIGHT - 1,
            cursor_col: 0,
            cursor_row: 0,
        }
    }

    /// Apply a captured transaction to the framebuffer. If the transaction
    /// is a 0x2C that exactly fills the active window in one go, return a
    /// `WindowWrite` describing it (handy for dumping individual glyphs).
    pub fn apply(&mut self, tx: &Frame) -> Option<WindowWrite> {
        match tx.cmd {
            0x2A => {
                if tx.data.len() >= 4 {
                    let cs = (tx.data[0] & 0xFF) << 8 | (tx.data[1] & 0xFF);
                    let ce = (tx.data[2] & 0xFF) << 8 | (tx.data[3] & 0xFF);
                    self.col_start = cs;
                    self.col_end = ce;
                    self.cursor_col = cs;
                    self.cursor_row = self.row_start;
                }
                None
            }
            0x2B => {
                if tx.data.len() >= 4 {
                    let rs = (tx.data[0] & 0xFF) << 8 | (tx.data[1] & 0xFF);
                    let re = (tx.data[2] & 0xFF) << 8 | (tx.data[3] & 0xFF);
                    self.row_start = rs;
                    self.row_end = re;
                    self.cursor_col = self.col_start;
                    self.cursor_row = rs;
                }
                None
            }
            0x2C | 0x3C => {
                // MEMORY_WRITE or MEMORY_WRITE_CONTINUE.
                if tx.cmd == 0x2C {
                    self.cursor_col = self.col_start;
                    self.cursor_row = self.row_start;
                }
                for &px in &tx.data {
                    if self.cursor_col < WIDTH && self.cursor_row < HEIGHT {
                        let idx =
                            self.cursor_row as usize * WIDTH as usize + self.cursor_col as usize;
                        self.pixels[idx] = px;
                    }
                    self.cursor_col += 1;
                    if self.cursor_col > self.col_end {
                        self.cursor_col = self.col_start;
                        self.cursor_row += 1;
                        if self.cursor_row > self.row_end {
                            self.cursor_row = self.row_start;
                        }
                    }
                }

                if tx.cmd == 0x2C
                    && self.col_end >= self.col_start
                    && self.row_end >= self.row_start
                {
                    let w = self.col_end - self.col_start + 1;
                    let h = self.row_end - self.row_start + 1;
                    if tx.data.len() == w as usize * h as usize {
                        return Some(WindowWrite {
                            x: self.col_start,
                            y: self.row_start,
                            w,
                            h,
                            pixels: tx.data.clone(),
                        });
                    }
                }
                None
            }
            _ => None,
        }
    }

    /// Convert the RGB565 framebuffer to RGBA8 for egui display.
    /// The panel scans left-to-right top-to-bottom but is physically
    /// mounted upside-down on the target device, so we rotate 180° here.
    pub fn to_rgba8(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.pixels.len() * 4);
        for &px in self.pixels.iter().rev() {
            let r5 = ((px >> 11) & 0x1F) as u8;
            let g6 = ((px >> 5) & 0x3F) as u8;
            let b5 = (px & 0x1F) as u8;
            out.push((r5 << 3) | (r5 >> 2));
            out.push((g6 << 2) | (g6 >> 4));
            out.push((b5 << 3) | (b5 >> 2));
            out.push(0xFF);
        }
        out
    }
}
