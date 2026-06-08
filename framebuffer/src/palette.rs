//! Fixed 16-entry RGB565 palette and nearest-colour mapping.
//!
//! The panel uses a tiny set of UI colours: surveyed over a real capture
//! (`reference/goodrun/run2.bin`) only 7 colours account for 100% of pixels
//! written (black bg, green/red/yellow/orange panels, white text, grey unit
//! labels). We bake those as the default palette and reserve one **misc** slot
//! to catch any colour that isn't close to a known entry — so unexpected
//! colours show up as an obvious marker rather than silently snapping to a
//! real panel colour.

/// Number of palette entries (4 bits per pixel).
pub const PALETTE_LEN: usize = 16;

/// Slot used for any RGB565 that isn't within [`MATCH_THRESHOLD`] of a real
/// entry. Rendered as magenta so stray colours are visually obvious.
pub const MISC_INDEX: u8 = 15;

/// Squared-distance threshold (in 5/6/5 component units, summed) under which a
/// colour is considered a match for a palette entry. Beyond this → misc slot.
/// The real panel colours are far apart, so a generous threshold still keeps
/// genuine noise out.
const MATCH_THRESHOLD: u32 = 64;

/// A 16-entry RGB565 colour table.
#[derive(Clone, Copy)]
pub struct Palette {
    pub colors: [u16; PALETTE_LEN],
}

impl Palette {
    /// RGB565 colour for a nibble index (0..16).
    pub fn color(&self, idx: u8) -> u16 {
        self.colors[(idx as usize) & (PALETTE_LEN - 1)]
    }

    /// Nearest palette index for an RGB565 colour. Returns [`MISC_INDEX`] if no
    /// entry is within [`MATCH_THRESHOLD`].
    pub fn nearest(&self, rgb565: u16) -> u8 {
        let (tr, tg, tb) = unpack(rgb565);
        let mut best = MISC_INDEX;
        let mut best_d = u32::MAX;
        for (i, &c) in self.colors.iter().enumerate() {
            if i as u8 == MISC_INDEX {
                continue; // never auto-match the misc slot
            }
            let (r, g, b) = unpack(c);
            let dr = tr as i32 - r as i32;
            let dg = tg as i32 - g as i32;
            let db = tb as i32 - b as i32;
            let d = (dr * dr + dg * dg + db * db) as u32;
            if d < best_d {
                best_d = d;
                best = i as u8;
            }
        }
        if best_d <= MATCH_THRESHOLD {
            best
        } else {
            MISC_INDEX
        }
    }
}

/// Split RGB565 into its 5/6/5 components.
fn unpack(px: u16) -> (u8, u8, u8) {
    (
        ((px >> 11) & 0x1F) as u8,
        ((px >> 5) & 0x3F) as u8,
        (px & 0x1F) as u8,
    )
}

/// Default palette baked from the panel survey. Entries 0..7 are the real
/// colours; 8..15 are spare (set to black) except slot 15 = misc (magenta).
pub const DEFAULT_PALETTE: Palette = Palette {
    colors: [
        0x0000, // 0  black      (background)
        0x07e0, // 1  green      (CO2 panel)
        0xffe0, // 2  yellow
        0xffff, // 3  white      (text)
        0xf800, // 4  red        (PM2.5 panel)
        0xfd20, // 5  orange
        0x8410, // 6  grey       (unit labels)
        0x0000, // 7  spare
        0x0000, // 8  spare
        0x0000, // 9  spare
        0x0000, // 10 spare
        0x0000, // 11 spare
        0x0000, // 12 spare
        0x0000, // 13 spare
        0x0000, // 14 spare
        0xf81f, // 15 misc       (magenta — unexpected colours)
    ],
};
