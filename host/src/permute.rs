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
//! including which captured signal is the cmd/data framing bit. Both
//! the Pico and the current F103 rig frame on DC (DC pulses low per
//! command byte, high for params/pixels — standard 8080); CS is not
//! used for framing.

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
/// (see firmware-stm32/README.md). GPIOB is the self-sufficient port:
/// DC + low byte DB0..DB7 + the top two red (R4/R3) and green (G5/G4)
/// bits. GPIOA holds WR plus the lower colour-refinement bits and CS.
///   PA0        → WR (timer ETR, not part of the sample)
///   PA1        → DB8  (G3)          (sample bit 1)
///   PA2..PA4   → DB11..DB13 (R0..R2)(sample bits 2..4)
///   PA5        → CS  (unused)       (sample bit 5)
///   PB0..PB1   → DB14..DB15 (R3,R4) (sample bits 16..17)
///   PB5..PB12  → DB0..DB7           (sample bits 21..28)
///   PB13..PB14 → DB9..DB10 (G4,G5)  (sample bits 29..30)
///   PB15       → DC                 (sample bit 31)  ← framing signal
pub fn permute_f103(sample: u32) -> (u16, bool) {
    let pa = sample as u16;
    let pb = (sample >> 16) as u16;
    let data = ((pb >> 5) & 0x007F)              // DB0..DB6   ← PB5..PB11
        | (((pb >> 12) & 0x0001) << 7)           // DB7        ← PB12
        | (((pa >> 1) & 0x0001) << 8)            // DB8        ← PA1
        | (((pb >> 13) & 0x0001) << 9)           // DB9        ← PB13
        | (((pb >> 14) & 0x0001) << 10)          // DB10       ← PB14
        | (((pa >> 2) & 0x0007) << 11)           // DB11..DB13 ← PA2..PA4
        | ((pb & 0x0001) << 14)                  // DB14       ← PB0
        | (((pb >> 1) & 0x0001) << 15);          // DB15       ← PB1
    let is_data = pb & (1 << 15) != 0;           // DC         ← PB15
    (data, is_data)
}
