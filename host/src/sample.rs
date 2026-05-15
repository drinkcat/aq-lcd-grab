//! A single bus sample as captured by the Pico's PIO state machine.

#[derive(Copy, Clone, Debug)]
pub struct Sample {
    pub data: u16,
    pub dc: bool,
    pub cs: bool,
}

impl Sample {
    /// Decode the 18-bit packed value emitted by the firmware.
    ///
    /// The firmware's [pio_capture.rs] comments call bit 16 "DC" and bit 17
    /// "CS", but live captures from the target only make protocol sense if
    /// those are swapped — strongly suggesting the FFC adapter's DC and CS
    /// pins are wired to GPIO 17 and GPIO 16 respectively (rather than
    /// 16/17). We fix it here in software rather than in the firmware so
    /// we don't have to re-flash.
    ///
    /// Layout in the captured u32:
    ///
    ///     bit  17 16 15 ............... 0
    ///          DC CS DB15 ............ DB0
    pub fn from_raw(raw: u32) -> Self {
        Self {
            data: (raw & 0xFFFF) as u16,
            cs: (raw >> 16) & 1 != 0,
            dc: (raw >> 17) & 1 != 0,
        }
    }
}
