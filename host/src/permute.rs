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
