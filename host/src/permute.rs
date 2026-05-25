//! Board-specific permutation: a raw `(pa | pb<<16)` sample → logical
//! `(data, is_data)`.
//!
//! Each capture board picks its own way to fan the 8080 bus across the
//! MCU's GPIO ports — see `docs/pcb_spec.md` §Q17. The wire protocol
//! ships the raw packed sample so the host can apply the right
//! permutation without the firmware having to know.
//!
//! Bit layout of `sample`: bits 0..15 = `GPIOA->IDR`, bits 16..31 =
//! `GPIOB->IDR`. Per-board `permute_*` functions know the routing —
//! including which captured signal is the cmd/data framing bit
//! (Pico uses DC, F103/target uses CS because the target's PIC32 holds DC
//! high and pulses CS instead).

/// Pico 2 W layout (firmware/src/main.rs + firmware/src/pio_capture.rs):
///   GPIO  0..15 → DB0..DB15
///   GPIO 16     → CS  (sample bit 16)
///   GPIO 17     → DC  (sample bit 17) ← framing signal
///
/// Verified by capturing the target's startup sequence: DC really does
/// pulse low for each command byte (0x2A SET_COL, 0x2B SET_ROW, 0x2C
/// MEM_WRITE, etc.), high for parameters and pixel data — standard
/// 8080.
pub fn permute_pico(sample: u32) -> (u16, bool) {
    let data = sample as u16;
    let is_data = sample & (1 << 17) != 0;
    (data, is_data)
}

/// F103 capture board layout, Blue Pill bench rig
/// (see firmware-stm32/README.md):
///   PA0        → WR (timer ETR, not part of the sample)
///   PA1..PA7   → DB0..DB6      (sample bits 1..7)
///   PB0..PB1   → DB7..DB8      (sample bits 16..17)
///   PB5..PB9   → DB11..DB15    (sample bits 21..25)
///   PB10..PB11 → DB9..DB10     (sample bits 26..27)
///   PB12       → DC            (sample bit 28)  ← framing signal
///   PB13       → CS            (sample bit 29)
pub fn permute_f103(sample: u32) -> (u16, bool) {
    let pa = sample as u16;
    let pb = (sample >> 16) as u16;
    let data = ((pa >> 1) & 0x007F)              // DB0..DB6   ← PA1..PA7
        | ((pb & 0x0003) << 7)                   // DB7..DB8   ← PB0..PB1
        | (((pb >> 10) & 0x0003) << 9)           // DB9..DB10  ← PB10..PB11
        | (((pb >> 5) & 0x001F) << 11);          // DB11..DB15 ← PB5..PB9
    let is_data = pb & (1 << 12) != 0;
    (data, is_data)
}
