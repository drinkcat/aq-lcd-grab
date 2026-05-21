//! Wire protocol encoder (firmware -> host).
//!
//! See `docs/wire_protocol.md` for the full spec. This module turns a
//! stream of paired `(pa, pb)` samples into a byte sequence of
//! tag=0x01 (block-of-unique) and tag=0x02 (run) frames, with helpers
//! for tag=0xFD (overrun) and tag=0xFE (log).
//!
//! The encoder is stateful: it accumulates a pending run AND a pending
//! block-of-unique, flushing whichever rule fires first. `flush()` must
//! be called whenever streaming transitions to STOPPED or to handle
//! end-of-burst.

pub const TAG_BLOCK: u8 = 0x01;
pub const TAG_RUN: u8 = 0x02;
pub const TAG_OVERRUN: u8 = 0xFD;
pub const TAG_LOG: u8 = 0xFE;
pub const TAG_STARTED: u8 = 0xFB;
pub const TAG_STOPPED: u8 = 0xFC;

pub const HOST_CMD_START: u8 = 0x01;
pub const HOST_CMD_STOP: u8 = 0x02;

/// Maximum samples per tag=0x01 block or tag=0x02 run.
const MAX_RUN: u8 = 255;
const MAX_BLOCK: usize = 255;

/// Byte sink: where the encoder pushes wire bytes. Implementor decides
/// what to do with overflow (drop, block, count).
pub trait Sink {
    /// Push one byte. Return true if accepted, false if dropped (full).
    fn push(&mut self, b: u8) -> bool;
}

/// Encoder state.
pub struct Encoder {
    /// Run-in-progress: same `(pa, pb)` repeated `run_len` times.
    /// `run_len` ∈ [0, MAX_RUN]; 0 means "no run pending".
    run_pa: u16,
    run_pb: u16,
    run_len: u8,
    /// Block-of-unique-in-progress: up to MAX_BLOCK distinct samples
    /// that haven't been flushed yet. Stored as flat `(pa,pb)` pairs.
    block: [u8; 4 * MAX_BLOCK],
    block_n: usize,
}

impl Default for Encoder {
    fn default() -> Self {
        Self {
            run_pa: 0,
            run_pb: 0,
            run_len: 0,
            block: [0; 4 * MAX_BLOCK],
            block_n: 0,
        }
    }
}

impl Encoder {
    /// Hard-reset internal state. Used on STOP.
    pub fn reset(&mut self) {
        self.run_len = 0;
        self.block_n = 0;
    }

    /// Feed one sample. May flush 0, 1, or 2 frames to `sink`.
    pub fn feed<S: Sink>(&mut self, pa: u16, pb: u16, sink: &mut S) {
        // If a run is in progress and this sample extends it, just bump.
        if self.run_len > 0 && pa == self.run_pa && pb == self.run_pb {
            self.run_len += 1;
            if self.run_len == MAX_RUN {
                self.flush_run(sink);
            }
            return;
        }

        // Not extending a run. Two sub-cases.

        // (a) We have a run of length ≥ 2 to flush. Flush it as tag=0x02
        // and start fresh with the new sample as the first block entry.
        if self.run_len >= 2 {
            self.flush_run(sink);
            self.push_to_block(pa, pb, sink);
            return;
        }

        // (b) Run of length 1 (or zero). That single sample, if it
        // exists, joins the block; then the new sample joins.
        if self.run_len == 1 {
            let prev_pa = self.run_pa;
            let prev_pb = self.run_pb;
            self.run_len = 0;
            self.push_to_block(prev_pa, prev_pb, sink);
        }

        // New sample becomes the seed of a new run-of-1.
        self.run_pa = pa;
        self.run_pb = pb;
        self.run_len = 1;
    }

    /// Push one sample into the block buffer. If the block fills, flush
    /// it as a tag=0x01 frame.
    fn push_to_block<S: Sink>(&mut self, pa: u16, pb: u16, sink: &mut S) {
        let off = self.block_n * 4;
        self.block[off] = pa as u8;
        self.block[off + 1] = (pa >> 8) as u8;
        self.block[off + 2] = pb as u8;
        self.block[off + 3] = (pb >> 8) as u8;
        self.block_n += 1;
        if self.block_n == MAX_BLOCK {
            self.flush_block(sink);
        }
    }

    fn flush_block<S: Sink>(&mut self, sink: &mut S) {
        if self.block_n == 0 {
            return;
        }
        sink.push(TAG_BLOCK);
        sink.push(self.block_n as u8);
        let bytes = self.block_n * 4;
        for &b in &self.block[..bytes] {
            sink.push(b);
        }
        self.block_n = 0;
    }

    fn flush_run<S: Sink>(&mut self, sink: &mut S) {
        if self.run_len == 0 {
            return;
        }
        if self.run_len == 1 {
            // A lone run-of-1 belongs in a block, not a tag=0x02 frame.
            let pa = self.run_pa;
            let pb = self.run_pb;
            self.run_len = 0;
            self.push_to_block(pa, pb, sink);
            return;
        }
        // run_len >= 2 → emit as tag=0x02.
        sink.push(TAG_RUN);
        sink.push(self.run_len);
        sink.push(self.run_pa as u8);
        sink.push((self.run_pa >> 8) as u8);
        sink.push(self.run_pb as u8);
        sink.push((self.run_pb >> 8) as u8);
        self.run_len = 0;
    }

    /// Flush whatever is accumulated. Call at end-of-burst or on STOP.
    pub fn flush<S: Sink>(&mut self, sink: &mut S) {
        // Order matters: any pending run is *older* than the block, so
        // flush it first to keep the wire-stream temporally ordered.
        // (Actually, by construction the block is always more recent
        // than a leftover run, so flush block last.)
        self.flush_run(sink);
        self.flush_block(sink);
    }
}

/// Encode a tag=0xFD overrun frame.
pub fn encode_overrun<S: Sink>(dropped: u32, sink: &mut S) {
    sink.push(TAG_OVERRUN);
    let b = dropped.to_le_bytes();
    sink.push(b[0]);
    sink.push(b[1]);
    sink.push(b[2]);
    sink.push(b[3]);
}

/// Encode a tag=0xFE log frame. `msg` is truncated to 256 bytes.
pub fn encode_log<S: Sink>(msg: &str, sink: &mut S) {
    let bytes = msg.as_bytes();
    let len = bytes.len().min(256);
    sink.push(TAG_LOG);
    sink.push(len as u8);
    sink.push((len >> 8) as u8);
    for &b in &bytes[..len] {
        sink.push(b);
    }
}

/// Encode tag=0xFB STARTED ack.
pub fn encode_started<S: Sink>(sink: &mut S) {
    sink.push(TAG_STARTED);
}

/// Encode tag=0xFC STOPPED ack.
pub fn encode_stopped<S: Sink>(sink: &mut S) {
    sink.push(TAG_STOPPED);
}
