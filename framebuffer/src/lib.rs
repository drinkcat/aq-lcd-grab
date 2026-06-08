//! Reconstruct the panel framebuffer by replaying the display-bus
//! `(data, is_data)` stream — the same level the wire decoder and glyph
//! decoder operate at.
//!
//! The ILI9488/ST7796 model:
//! - `SET_COLUMN_ADDRESS (0x2A)`: 4 param words (start_h, start_l, end_h, end_l).
//! - `SET_ROW_ADDRESS (0x2B)`: 4 param words, same layout.
//! - `MEMORY_WRITE (0x2C)` / `..._CONTINUE (0x3C)`: RGB565 pixel words; the
//!   panel auto-increments column then row within the active window.
//!
//! On a 16-bit 8080 bus each address parameter byte is in the low 8 bits of a
//! bus word, so an address transaction is 4 *words*. Pixels are 16-bit RGB565.
//!
//! Pixel storage is pluggable via [`PixelStore`]:
//! - [`Rgb565Store`] keeps full 16-bit fidelity (host viewer) — one `u16` per
//!   pixel (~300 KiB for 320×480).
//! - [`Palette4Store`] keeps 4 bits/pixel into a fixed [`Palette`] (~75 KiB),
//!   mapping each written RGB565 to the nearest palette entry. For RAM-bound
//!   targets (ESP32).
//!
//! The crate is pure `no_std` and allocation-free: the pixel buffer is always
//! caller-provided (a `StaticCell` slice on embedded, a `Box`/`Vec` on host).

#![cfg_attr(not(test), no_std)]

pub const WIDTH: u16 = 320;
pub const HEIGHT: u16 = 480;
pub const PIXELS: usize = WIDTH as usize * HEIGHT as usize;

mod palette;
pub use palette::{Palette, DEFAULT_PALETTE, MISC_INDEX};

/// Backing pixel storage. Pixels are addressed by linear index
/// `row * WIDTH + col`. Implementations decide how to store the colour.
pub trait PixelStore {
    /// Number of pixels the store holds. Must be at least [`PIXELS`].
    fn len(&self) -> usize;
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
    /// Store an RGB565 colour at `idx` (mapping/quantising as needed).
    fn set(&mut self, idx: usize, rgb565: u16);
    /// Read back the RGB565 colour at `idx` (de-quantising if needed).
    fn get(&self, idx: usize) -> u16;
}

/// Full-fidelity RGB565 storage: one `u16` per pixel.
pub struct Rgb565Store<'a> {
    pub pixels: &'a mut [u16],
}

impl<'a> Rgb565Store<'a> {
    /// `pixels` must be at least [`PIXELS`] long.
    pub fn new(pixels: &'a mut [u16]) -> Self {
        assert!(pixels.len() >= PIXELS, "framebuffer too small");
        Self { pixels }
    }
}

impl PixelStore for Rgb565Store<'_> {
    fn len(&self) -> usize {
        self.pixels.len()
    }
    fn set(&mut self, idx: usize, rgb565: u16) {
        self.pixels[idx] = rgb565;
    }
    fn get(&self, idx: usize) -> u16 {
        self.pixels[idx]
    }
}

/// 4-bits-per-pixel storage into a fixed 16-entry [`Palette`]. Two pixels per
/// byte; the low nibble is the even pixel. `set` maps the RGB565 colour to the
/// nearest palette entry (within a threshold; otherwise the palette's misc
/// slot).
pub struct Palette4Store<'a> {
    /// Packed nibbles: `bytes[i]` holds pixel `2*i` (low) and `2*i+1` (high).
    /// Must be at least `PIXELS / 2` long.
    pub bytes: &'a mut [u8],
    pub palette: Palette,
}

impl<'a> Palette4Store<'a> {
    /// `bytes` must be at least `PIXELS / 2` long.
    pub fn new(bytes: &'a mut [u8], palette: Palette) -> Self {
        assert!(bytes.len() >= PIXELS / 2, "palette framebuffer too small");
        Self { bytes, palette }
    }

    /// Raw nibble index at `idx` (0..16).
    pub fn index_at(&self, idx: usize) -> u8 {
        let byte = self.bytes[idx / 2];
        if idx & 1 == 0 {
            byte & 0x0F
        } else {
            byte >> 4
        }
    }

    fn set_index(&mut self, idx: usize, pal_idx: u8) {
        let b = &mut self.bytes[idx / 2];
        if idx & 1 == 0 {
            *b = (*b & 0xF0) | (pal_idx & 0x0F);
        } else {
            *b = (*b & 0x0F) | (pal_idx << 4);
        }
    }
}

impl PixelStore for Palette4Store<'_> {
    fn len(&self) -> usize {
        self.bytes.len() * 2
    }
    fn set(&mut self, idx: usize, rgb565: u16) {
        let pal_idx = self.palette.nearest(rgb565);
        self.set_index(idx, pal_idx);
    }
    fn get(&self, idx: usize) -> u16 {
        self.palette.color(self.index_at(idx))
    }
}

// ---- 8080 command framing ----

#[derive(Default, Clone, Copy)]
enum Cmd {
    #[default]
    None,
    /// Collecting 4 address words for SET_COLUMN_ADDRESS (0x2A).
    ColAddr { buf: [u16; 4], n: u8 },
    /// Collecting 4 address words for SET_ROW_ADDRESS (0x2B).
    RowAddr { buf: [u16; 4], n: u8 },
    /// Receiving pixel data for MEMORY_WRITE (0x2C / 0x3C).
    MemWrite,
}

/// Framebuffer reconstructor over a pluggable [`PixelStore`].
pub struct Framebuffer<S: PixelStore> {
    store: S,
    col_start: u16,
    col_end: u16,
    row_start: u16,
    row_end: u16,
    cursor_col: u16,
    cursor_row: u16,
    cmd: Cmd,
}

impl<S: PixelStore> Framebuffer<S> {
    pub fn new(store: S) -> Self {
        assert!(store.len() >= PIXELS, "store too small");
        Self {
            store,
            col_start: 0,
            col_end: WIDTH - 1,
            row_start: 0,
            row_end: HEIGHT - 1,
            cursor_col: 0,
            cursor_row: 0,
            cmd: Cmd::None,
        }
    }

    pub fn store(&self) -> &S {
        &self.store
    }

    /// Feed one permuted bus sample `(data, is_data)`, doing 8080 framing
    /// internally (mirrors the glyph decoder, so both can share one feed loop).
    pub fn feed(&mut self, data: u16, is_data: bool) {
        if !is_data {
            self.cmd = match data as u8 {
                0x2A => Cmd::ColAddr { buf: [0; 4], n: 0 },
                0x2B => Cmd::RowAddr { buf: [0; 4], n: 0 },
                0x2C => {
                    self.cursor_col = self.col_start;
                    self.cursor_row = self.row_start;
                    Cmd::MemWrite
                }
                // MEMORY_WRITE_CONTINUE keeps the cursor where it was.
                0x3C => Cmd::MemWrite,
                _ => Cmd::None,
            };
            return;
        }

        match &mut self.cmd {
            Cmd::ColAddr { buf, n } => {
                if (*n as usize) < 4 {
                    buf[*n as usize] = data & 0xFF;
                    *n += 1;
                    if *n == 4 {
                        self.col_start = (buf[0] << 8) | buf[1];
                        self.col_end = (buf[2] << 8) | buf[3];
                    }
                }
            }
            Cmd::RowAddr { buf, n } => {
                if (*n as usize) < 4 {
                    buf[*n as usize] = data & 0xFF;
                    *n += 1;
                    if *n == 4 {
                        self.row_start = (buf[0] << 8) | buf[1];
                        self.row_end = (buf[2] << 8) | buf[3];
                    }
                }
            }
            Cmd::MemWrite => self.write_pixel(data),
            Cmd::None => {}
        }
    }

    fn write_pixel(&mut self, px: u16) {
        if self.cursor_col < WIDTH && self.cursor_row < HEIGHT {
            let idx = self.cursor_row as usize * WIDTH as usize + self.cursor_col as usize;
            self.store.set(idx, px);
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

    /// RGB565 colour at display-space pixel `(x, y)` *before* the 180° rotation
    /// (raw panel space). Mostly for tests.
    pub fn pixel(&self, x: u16, y: u16) -> u16 {
        self.store.get(y as usize * WIDTH as usize + x as usize)
    }
}

// ---- BMP export ----

/// Total byte length of the 24-bit BMP produced by [`write_bmp`].
pub const BMP_LEN: usize = BMP_HEADER_LEN + PIXELS * 3;
/// Length of the BMP header (BITMAPFILEHEADER + BITMAPINFOHEADER).
pub const BMP_HEADER_LEN: usize = 54;

/// Fill `out[..BMP_HEADER_LEN]` with the 24-bit BMP header for this panel.
/// Returns the header bytes. Useful for streaming the image: write the header,
/// then stream pixel rows (see [`bmp_row_bgr`]) without buffering the whole BMP.
pub fn bmp_header() -> [u8; BMP_HEADER_LEN] {
    let row_bytes = WIDTH as usize * 3;
    let pixel_data = row_bytes * HEIGHT as usize;
    let file_len = BMP_HEADER_LEN + pixel_data;
    let mut h = [0u8; BMP_HEADER_LEN];
    h[0] = b'B';
    h[1] = b'M';
    h[2..6].copy_from_slice(&(file_len as u32).to_le_bytes());
    h[10..14].copy_from_slice(&(BMP_HEADER_LEN as u32).to_le_bytes());
    h[14..18].copy_from_slice(&40u32.to_le_bytes()); // info header size
    h[18..22].copy_from_slice(&(WIDTH as i32).to_le_bytes());
    // Negative height = top-down rows (row 0 is the visual top). We emit pixels
    // in fully-reversed framebuffer order (the 180° rotation for the
    // upside-down mount), same order as `to_rgba8`; top-down then renders that
    // upright. Positive height would re-flip it (bottom-up) → upside down.
    h[22..26].copy_from_slice(&(-(HEIGHT as i32)).to_le_bytes());
    h[26..28].copy_from_slice(&1u16.to_le_bytes()); // planes
    h[28..30].copy_from_slice(&24u16.to_le_bytes()); // bpp
    h[34..38].copy_from_slice(&(pixel_data as u32).to_le_bytes());
    h
}

/// Write a 24-bit (BGR) top-down BMP of the framebuffer into `out`, applying
/// the 180° rotation (panel is mounted upside-down). Returns the number of
/// bytes written (always [`BMP_LEN`]). `out` must be at least [`BMP_LEN`].
///
/// A 24-bit BMP needs no palette table and is rendered correctly by every
/// browser. Rows are 320×3 = 960 bytes — already a multiple of 4, so no
/// per-row padding. Allocation-free.
pub fn write_bmp<S: PixelStore>(fb: &Framebuffer<S>, out: &mut [u8]) -> usize {
    assert!(out.len() >= BMP_LEN, "bmp output buffer too small");
    out[..BMP_HEADER_LEN].copy_from_slice(&bmp_header());
    let mut o = BMP_HEADER_LEN;
    o += bmp_pixels_bgr(fb, 0, PIXELS, &mut out[BMP_HEADER_LEN..]);
    o
}

/// Write the BGR pixel bytes for source-pixel range `[start, start + count)`
/// into `out` (must hold `count * 3` bytes), in BMP order. Pixels are emitted
/// in fully-reversed source order (the 180° rotation for the upside-down mount);
/// paired with the top-down (negative-height) header this renders upright.
/// Streaming `start` from 0..PIXELS in chunks yields the full pixel-data
/// section. Returns bytes written.
pub fn bmp_pixels_bgr<S: PixelStore>(
    fb: &Framebuffer<S>,
    start: usize,
    count: usize,
    out: &mut [u8],
) -> usize {
    assert!(out.len() >= count * 3, "bmp pixel chunk too small");
    let mut o = 0;
    for i in start..start + count {
        // Reverse-index into the framebuffer (180° rotation for the mount).
        let src = PIXELS - 1 - i;
        let (r, g, b) = rgb565_to_rgb888(fb.store.get(src));
        out[o] = b;
        out[o + 1] = g;
        out[o + 2] = r;
        o += 3;
    }
    o
}

// ---- 4bpp palettized BMP export ----
//
// The panel uses ≤16 colours, so a 4-bit indexed BMP is ~6× smaller than the
// 24-bit form (76 800 B of pixels + a tiny colour table vs 460 800 B) — a big
// win over WiFi. The framebuffer already stores 4bpp palette indices, so this
// is close to a memcpy.

use palette::PALETTE_LEN;

/// 4bpp BMP header length: file header (14) + info header (40) + 16-entry
/// colour table (16 × 4 BGRA bytes).
pub const BMP4_HEADER_LEN: usize = 14 + 40 + PALETTE_LEN * 4;
/// 4bpp BMP pixel-data length: WIDTH/2 bytes per row (320/2 = 160, already a
/// multiple of 4 so no row padding) × HEIGHT.
pub const BMP4_PIXELS_LEN: usize = (WIDTH as usize / 2) * HEIGHT as usize;
/// Total 4bpp BMP length.
pub const BMP4_LEN: usize = BMP4_HEADER_LEN + BMP4_PIXELS_LEN;

/// Fill `out[..BMP4_HEADER_LEN]` with a 4bpp BMP header + colour table for the
/// given palette. Top-down (negative height) so reversed pixel order renders
/// upright. Returns the header bytes.
pub fn bmp4_header(palette: &Palette) -> [u8; BMP4_HEADER_LEN] {
    let mut h = [0u8; BMP4_HEADER_LEN];
    h[0] = b'B';
    h[1] = b'M';
    h[2..6].copy_from_slice(&(BMP4_LEN as u32).to_le_bytes());
    h[10..14].copy_from_slice(&(BMP4_HEADER_LEN as u32).to_le_bytes());
    h[14..18].copy_from_slice(&40u32.to_le_bytes()); // info header size
    h[18..22].copy_from_slice(&(WIDTH as i32).to_le_bytes());
    h[22..26].copy_from_slice(&(-(HEIGHT as i32)).to_le_bytes()); // top-down
    h[26..28].copy_from_slice(&1u16.to_le_bytes()); // planes
    h[28..30].copy_from_slice(&4u16.to_le_bytes()); // bpp
    h[34..38].copy_from_slice(&(BMP4_PIXELS_LEN as u32).to_le_bytes());
    h[46..50].copy_from_slice(&(PALETTE_LEN as u32).to_le_bytes()); // biClrUsed
    // Colour table: BGRA per entry.
    for (i, &c) in palette.colors.iter().enumerate() {
        let (r, g, b) = rgb565_to_rgb888(c);
        let o = 54 + i * 4;
        h[o] = b;
        h[o + 1] = g;
        h[o + 2] = r;
        h[o + 3] = 0;
    }
    h
}

/// Write 4bpp pixel bytes for output-pixel range `[start, start + count)` into
/// `out` (≥ `count / 2` bytes; `start`/`count` must be even). Pixels are emitted
/// in fully-reversed framebuffer order (180° rotation); with the top-down header
/// this renders upright. The high nibble is the left (lower-index) pixel, per
/// the BMP 4bpp convention. Returns bytes written.
pub fn bmp4_pixels(store: &Palette4Store<'_>, start: usize, count: usize, out: &mut [u8]) -> usize {
    debug_assert!(start % 2 == 0 && count % 2 == 0);
    let mut o = 0;
    let mut i = start;
    while i < start + count {
        let hi = store.index_at(PIXELS - 1 - i);
        let lo = store.index_at(PIXELS - 1 - (i + 1));
        out[o] = (hi << 4) | (lo & 0x0F);
        o += 1;
        i += 2;
    }
    o
}

/// Length of the RGBA8 buffer produced by [`write_rgba8`].
pub const RGBA8_LEN: usize = PIXELS * 4;

/// Write the framebuffer as RGBA8 (for egui display), rotated 180° to match the
/// upside-down panel mount. `out` must be at least [`RGBA8_LEN`]. Allocation-free.
pub fn write_rgba8<S: PixelStore>(fb: &Framebuffer<S>, out: &mut [u8]) -> usize {
    assert!(out.len() >= RGBA8_LEN, "rgba8 output buffer too small");
    let mut o = 0;
    for src in (0..PIXELS).rev() {
        let (r, g, b) = rgb565_to_rgb888(fb.store.get(src));
        out[o] = r;
        out[o + 1] = g;
        out[o + 2] = b;
        out[o + 3] = 0xFF;
        o += 4;
    }
    RGBA8_LEN
}

/// Expand an RGB565 word to 8-bit-per-channel RGB.
pub fn rgb565_to_rgb888(px: u16) -> (u8, u8, u8) {
    let r5 = ((px >> 11) & 0x1F) as u8;
    let g6 = ((px >> 5) & 0x3F) as u8;
    let b5 = (px & 0x1F) as u8;
    (
        (r5 << 3) | (r5 >> 2),
        (g6 << 2) | (g6 >> 4),
        (b5 << 3) | (b5 >> 2),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    extern crate alloc;
    use alloc::vec;

    fn cmd(fb: &mut Framebuffer<Rgb565Store<'_>>, c: u8) {
        fb.feed(c as u16, false);
    }
    fn param(fb: &mut Framebuffer<Rgb565Store<'_>>, w: u16) {
        fb.feed(w, true);
    }

    /// Set the active window via 0x2A/0x2B (col/row inclusive).
    fn set_window(fb: &mut Framebuffer<Rgb565Store<'_>>, cs: u16, ce: u16, rs: u16, re: u16) {
        cmd(fb, 0x2A);
        param(fb, cs >> 8);
        param(fb, cs & 0xFF);
        param(fb, ce >> 8);
        param(fb, ce & 0xFF);
        cmd(fb, 0x2B);
        param(fb, rs >> 8);
        param(fb, rs & 0xFF);
        param(fb, re >> 8);
        param(fb, re & 0xFF);
    }

    #[test]
    fn bmp4_header_and_pixels_are_consistent() {
        let mut bytes = vec![0u8; PIXELS / 2];
        let mut store = Palette4Store::new(&mut bytes, DEFAULT_PALETTE);
        // Put a known colour at the very first framebuffer pixel (index 0) and a
        // different one at the last (index PIXELS-1).
        store.set(0, 0xf800); // red → palette index 4
        store.set(PIXELS - 1, 0x07e0); // green → palette index 1

        let header = bmp4_header(&store.palette);
        assert_eq!(&header[0..2], b"BM");
        assert_eq!(u16::from_le_bytes([header[28], header[29]]), 4); // 4 bpp
        assert_eq!(
            i32::from_le_bytes([header[22], header[23], header[24], header[25]]),
            -(HEIGHT as i32) // top-down
        );
        assert_eq!(header.len() + BMP4_PIXELS_LEN, BMP4_LEN);

        // First output pixel is reversed → framebuffer index PIXELS-1 (green=1);
        // it lands in the high nibble of the first byte.
        let mut px = vec![0u8; BMP4_PIXELS_LEN];
        let n = bmp4_pixels(&store, 0, PIXELS, &mut px);
        assert_eq!(n, BMP4_PIXELS_LEN);
        assert_eq!(px[0] >> 4, 1, "first output pixel should be green (idx 1)");
        // Last output pixel is framebuffer index 0 (red=4) → low nibble of last byte.
        assert_eq!(px[BMP4_PIXELS_LEN - 1] & 0x0F, 4, "last output pixel should be red (idx 4)");
    }

    #[test]
    fn fills_a_window_in_scan_order() {
        let mut buf = vec![0u16; PIXELS];
        let mut fb = Framebuffer::new(Rgb565Store::new(&mut buf));
        // 2x2 window at (10,20).
        set_window(&mut fb, 10, 11, 20, 21);
        cmd(&mut fb, 0x2C);
        param(&mut fb, 0x1111);
        param(&mut fb, 0x2222);
        param(&mut fb, 0x3333);
        param(&mut fb, 0x4444);
        assert_eq!(fb.pixel(10, 20), 0x1111);
        assert_eq!(fb.pixel(11, 20), 0x2222);
        assert_eq!(fb.pixel(10, 21), 0x3333);
        assert_eq!(fb.pixel(11, 21), 0x4444);
    }

    #[test]
    fn palette_store_round_trips_known_colors() {
        let mut bytes = vec![0u8; PIXELS / 2];
        let mut store = Palette4Store::new(&mut bytes, DEFAULT_PALETTE);
        // Known palette colours must round-trip exactly.
        for &c in &[0x0000u16, 0x07e0, 0xffe0, 0xffff, 0xf800, 0xfd20, 0x8410] {
            store.set(0, c);
            assert_eq!(store.get(0), c, "color {c:#06x} did not round-trip");
        }
    }

    #[test]
    fn palette_store_maps_unknown_to_misc() {
        let mut bytes = vec![0u8; PIXELS / 2];
        let mut store = Palette4Store::new(&mut bytes, DEFAULT_PALETTE);
        // A colour far from every known entry → misc slot.
        store.set(0, 0x001c); // a dark blue, not in the panel palette
        assert_eq!(store.index_at(0), MISC_INDEX);
    }

    #[test]
    fn palette_nibble_packing() {
        let mut bytes = vec![0u8; PIXELS / 2];
        let mut store = Palette4Store::new(&mut bytes, DEFAULT_PALETTE);
        store.set(0, 0x07e0); // green
        store.set(1, 0xf800); // red
        // Two distinct pixels share one byte without clobbering each other.
        assert_eq!(store.get(0), 0x07e0);
        assert_eq!(store.get(1), 0xf800);
    }

    #[test]
    fn bmp_has_valid_header_and_length() {
        let mut buf = vec![0u16; PIXELS];
        let mut fb = Framebuffer::new(Rgb565Store::new(&mut buf));
        set_window(&mut fb, 0, 0, 0, 0);
        cmd(&mut fb, 0x2C);
        param(&mut fb, 0xF800); // one red pixel at (0,0)

        let mut out = vec![0u8; BMP_LEN];
        let n = write_bmp(&fb, &mut out);
        assert_eq!(n, BMP_LEN);
        assert_eq!(&out[0..2], b"BM");
        assert_eq!(u32::from_le_bytes([out[2], out[3], out[4], out[5]]), BMP_LEN as u32);
        assert_eq!(u16::from_le_bytes([out[28], out[29]]), 24); // bpp
    }
}
