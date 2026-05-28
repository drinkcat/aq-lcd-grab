//! Byte sink: where the encoder pushes wire bytes.
//!
//! The encoder `push`es wire bytes one at a time and calls
//! [`Sink::flush`] to ensure the bytes pushed so far reach the wire.

pub trait Sink {
    /// Push one wire byte. Returns `true` if accepted, `false` if
    /// dropped (sink full).
    fn push(&mut self, b: u8) -> bool;

    /// Ensure everything pushed so far is sent out to the wire. Default:
    /// no-op, for a sink that pushes straight through.
    fn flush(&mut self) {}

    /// Push each byte of `bytes` in order.
    fn push_bytes(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.push(b);
        }
    }

    /// Push a `u16` in little-endian wire order.
    fn push_u16(&mut self, v: u16) {
        self.push_bytes(&v.to_le_bytes());
    }

    /// Push a `u32` in little-endian wire order.
    fn push_u32(&mut self, v: u32) {
        self.push_bytes(&v.to_le_bytes());
    }
}

/// A `Sink` that records the flat byte stream pushed to it — the exact
/// bytes that would hit the wire. Test-only.
#[cfg(test)]
#[derive(Debug, Default)]
pub struct VecSink {
    pub bytes: std::vec::Vec<u8>,
}

#[cfg(test)]
impl VecSink {
    pub fn new() -> Self {
        Self::default()
    }
}

#[cfg(test)]
impl Sink for VecSink {
    fn push(&mut self, b: u8) -> bool {
        self.bytes.push(b);
        true
    }
}
