//! Board-specific permutation: raw `(pa, pb)` port reads → logical
//! `(data, dc, cs)`.
//!
//! Each capture board picks its own way to fan the 8080 bus across the
//! MCU's GPIO ports — see `docs/pcb_spec.md` §Q17. The wire protocol
//! ships the raw `(pa, pb)` so the host can apply the right
//! permutation without the firmware having to know.

/// Pico 2 W layout (firmware/src/main.rs + firmware/src/pio_capture.rs):
///   GPIO  0..15 → DB0..DB15  — already in logical order, so pa = data
///   GPIO 16     → D/C        — ends up in pb bit 1
///   GPIO 17     → CS         — ends up in pb bit 0
pub fn permute_pico(pa: u16, pb: u16) -> (u16, bool, bool) {
    let data = pa;
    let cs = pb & 1 != 0;
    let dc = pb & 2 != 0;
    (data, dc, cs)
}

/// F103 capture board layout (firmware-stm32/src/capture.rs):
///   PA0..PA7   → DB0..DB7
///   PB0..PB1   → DB8..DB9
///   PB3..PB8   → DB10..DB15  (PB2 isn't exposed on the F103C8 package)
///   PB10       → DC
///   PB11       → CS
///
/// The PA half maps straight through; the PB data bits are stretched
/// across a 1-bit gap (PB2 is absent), so DB10..DB15 come from
/// PB3..PB8.
pub fn permute_f103(pa: u16, pb: u16) -> (u16, bool, bool) {
    let data = (pa & 0x00FF)                    // DB0..DB7  ← PA0..PA7
        | ((pb & 0x0003) << 8)                  // DB8..DB9  ← PB0..PB1
        | (((pb >> 3) & 0x003F) << 10);         // DB10..DB15 ← PB3..PB8
    let dc = pb & (1 << 10) != 0;
    let cs = pb & (1 << 11) != 0;
    (data, dc, cs)
}
