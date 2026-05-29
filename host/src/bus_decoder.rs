//! 8080 bus framing: `(data, is_data)` samples → command transactions.
//!
//! `is_data == false` marks a new command byte (low 8 bits of the bus
//! word). `is_data == true` contributes a data word to the in-flight
//! command's payload. The current transaction is emitted when the
//! next command-byte sample arrives.
//!
//! Whether DC or CS provides the framing signal is a per-board choice
//! made in `permute.rs` — on the Pico capture rig the panel's DC line
//! is what changes per byte; on the F103 board the capture target holds
//! DC high constantly and uses CS as the cmd/data discriminator
//! instead.

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

    /// Feed one sample. `is_data == false` means this byte is a new
    /// command — returns the previous transaction.
    pub fn feed(&mut self, data: u16, is_data: bool) -> Option<Frame> {
        if !is_data {
            // Command byte. Emit whatever was in flight.
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

    /// Feed `n` copies of the same sample efficiently.
    pub fn feed_run(&mut self, n: usize, data: u16, is_data: bool) -> Option<Frame> {
        if n == 0 {
            return None;
        }
        if !is_data {
            // n command-byte samples in a row — the same command
            // repeated. Only the first restarts framing; the rest are
            // redundant. Emit the previous tx once, then collapse.
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
