//! Stream decoder: bus samples -> transactions (cmd byte + data words).
//!
//! Mirror of `host/src/decoder.rs`, but `no_std` and fixed-capacity.
//! Boundary rule is DC-only (DC=0 starts a new command, DC=1 is data),
//! matching the 8080 spec. CS isn't used for framing because the live
//! captures show occasional CS=1 glitches mid-transfer.
//!
//! When the data buffer for a transaction fills, we flush it as a
//! `MEMORY_WRITE_CONTINUE` if the original command was a memory write
//! (so the host can chain them); otherwise we just drop further data
//! and warn — non-MW commands never need more than a few args.

use heapless::Vec;

use crate::proto::{CMD_MEMORY_WRITE_CONTINUE, MAX_DATA_WORDS};

const MEMORY_WRITE: u8 = 0x2C;

#[derive(Clone)]
pub struct Transaction {
    pub cmd: u8,
    pub data: Vec<u16, { MAX_DATA_WORDS as usize }>,
}

impl Transaction {
    pub fn new(cmd: u8) -> Self {
        Self {
            cmd,
            data: Vec::new(),
        }
    }
}

#[derive(Copy, Clone)]
pub struct Sample {
    pub data: u16,
    pub dc: bool,
    /// CS bit — captured but not used for framing.
    #[allow(dead_code)]
    pub cs: bool,
}

impl Sample {
    /// Decode the 18-bit packed value from the PIO FIFO.
    ///
    /// NOTE: live captures show that bits 16/17 in the captured u32 are
    /// CS/DC respectively (not DC/CS as the original pio_capture.rs
    /// comments suggest). The FFC adapter has DC and CS physically
    /// wired to GPIO 17 and GPIO 16. We compensate here instead of
    /// rewiring.
    pub fn from_raw(raw: u32) -> Self {
        Self {
            data: (raw & 0xFFFF) as u16,
            cs: (raw >> 16) & 1 != 0,
            dc: (raw >> 17) & 1 != 0,
        }
    }
}

#[derive(Default)]
pub struct Decoder {
    current: Option<Transaction>,
}

impl Decoder {
    /// Feed one sample. Returns a completed transaction if a boundary
    /// was crossed (either a new command, or the per-frame data buffer
    /// filled). The caller is expected to send/process it.
    pub fn feed(&mut self, s: Sample) -> Option<Transaction> {
        if !s.dc {
            // DC=0 → new command byte. Emit whatever was in flight.
            let new_tx = Transaction::new((s.data & 0xFF) as u8);
            return self.current.replace(new_tx);
        }

        // DC=1 → data word for the current transaction.
        let tx = self.current.as_mut()?;

        if tx.data.push(s.data).is_err() {
            // Capacity reached. Chain as MEMORY_WRITE_CONTINUE for memory
            // writes; for any other command, keep the original cmd so the
            // host doesn't mistake an orphan DC=1 burst (e.g. samples
            // captured after a missed 0x2C boundary) for valid pixels.
            let cont_cmd = match tx.cmd {
                MEMORY_WRITE | CMD_MEMORY_WRITE_CONTINUE => CMD_MEMORY_WRITE_CONTINUE,
                other => other,
            };
            let mut next = Transaction::new(cont_cmd);
            let _ = next.data.push(s.data);
            return self.current.replace(next);
        }

        None
    }
}
