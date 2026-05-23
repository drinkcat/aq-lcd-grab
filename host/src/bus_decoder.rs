//! 8080 bus framing: `(data, dc)` samples → command transactions.
//!
//! DC=0 marks a new command byte (low 8 bits of the bus word). DC=1
//! contributes a data word to the in-flight command's payload. The
//! current transaction is emitted when the next DC=0 sample arrives.
//!
//! CS isn't used for framing — live captures of the target show
//! occasional CS=1 glitches mid-transfer, so DC alone is the
//! authoritative boundary (same convention the original Pico-side
//! decoder used).

/// One framed 8080 transaction.
#[derive(Clone, Debug)]
pub struct Frame {
    pub cmd: u8,
    pub data: Vec<u16>,
}

#[derive(Default)]
pub struct BusDecoder {
    current: Option<Frame>,
}

impl BusDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed one `(data, dc)` sample. If this sample is a new command
    /// (DC=0), returns the previous transaction.
    pub fn feed(&mut self, data: u16, dc: bool) -> Option<Frame> {
        if !dc {
            // DC=0 → new command byte. Emit whatever was in flight.
            let next = Frame {
                cmd: (data & 0xFF) as u8,
                data: Vec::new(),
            };
            return self.current.replace(next);
        }
        if let Some(tx) = self.current.as_mut() {
            tx.data.push(data);
        }
        None
    }

    /// Feed `n` copies of the same sample efficiently. Equivalent to
    /// calling `feed(data, dc)` `n` times, but without growing the
    /// payload one push at a time.
    pub fn feed_run(&mut self, n: usize, data: u16, dc: bool) -> Option<Frame> {
        if n == 0 {
            return None;
        }
        if !dc {
            // n DC=0 samples in a row = the same command byte repeated.
            // Only the first edge restarts framing; the rest are redundant
            // re-issues of the same command with no payload. Emit the
            // previous tx once, then collapse the run.
            let next = Frame {
                cmd: (data & 0xFF) as u8,
                data: Vec::new(),
            };
            return self.current.replace(next);
        }
        if let Some(tx) = self.current.as_mut() {
            tx.data.reserve(n);
            for _ in 0..n {
                tx.data.push(data);
            }
        }
        None
    }
}
