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

/// F103 capture board layout, Blue Pill bench rig (firmware-stm32/src/capture.rs):
///   PA0        → WR (timer ETR, not part of the sample)
///   PA1..PA7   → DB1..DB7
///   PB0..PB1   → DB8..DB9
///   PB5..PB8   → DB12..DB15
///   PB9        → DB0  (relocated off PA0)
///   PB10       → DC
///   PB11       → CS
///   PB12..PB13 → DB10..DB11  (moved off PB3/PB4 to dodge JTAG)
///
/// DB0 wraps around to the PB half, and DB10..DB11 sit above the
/// control bits — so the PB→data extraction is in two non-contiguous
/// groups.
pub fn permute_f103(pa: u16, pb: u16) -> (u16, bool, bool) {
    let data = ((pb >> 9) & 0x0001)              // DB0        ← PB9
        | (pa & 0x00FE)                          // DB1..DB7   ← PA1..PA7
        | ((pb & 0x0003) << 8)                   // DB8..DB9   ← PB0..PB1
        | (((pb >> 12) & 0x0003) << 10)          // DB10..DB11 ← PB12..PB13
        | (((pb >> 5) & 0x000F) << 12);          // DB12..DB15 ← PB5..PB8
    let dc = pb & (1 << 10) != 0;
    let cs = pb & (1 << 11) != 0;
    (data, dc, cs)
}
