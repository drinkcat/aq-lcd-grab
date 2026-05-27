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
//!
//! This is byte-for-byte identical to `firmware-stm32/src/wire.rs` — the
//! two boards emit the same wire format so the host's parser is shared.
//! What varies per board is the *meaning* of `(pa, pb)`; the host applies
//! a board-specific permutation table to recover `(data, dc, cs)`.

pub const TAG_BLOCK: u8 = 0x01;
pub const TAG_RUN: u8 = 0x02;
pub const TAG_OVERRUN: u8 = 0xFD;
pub const TAG_LOG: u8 = 0xFE;
pub const TAG_STARTED: u8 = 0xFB;
pub const TAG_STOPPED: u8 = 0xFC;

pub const HOST_CMD_START: u8 = 0x01;
pub const HOST_CMD_STOP: u8 = 0x02;
pub const HOST_CMD_LOG_TEST: u8 = 0x03;
pub const HOST_CMD_STATS: u8 = 0x04;

/// Maximum samples per tag=0x01 block (u8 count) or tag=0x02 run
/// (u16 count — see wire_protocol.md). The Pico keeps the full 255-sample
/// block; the STM32 build trims its block to save RAM.
const MAX_RUN: u16 = u16::MAX;
const MAX_BLOCK: usize = 255;

/// Byte sink: where the encoder pushes wire bytes.
///
/// Frame atomicity is enforced via [`commit_frame`](Self::commit_frame):
/// the encoder pushes a frame's bytes one at a time, then calls
/// `commit_frame()` to signal "this frame is complete". A sink that can
/// fail half-way through a frame (e.g. a finite-capacity queue) must
/// buffer per-frame and discard the partial frame on commit if any push
/// failed. Without that, the host would parse the truncated frame and
/// desync.
pub trait Sink {
    /// Push one byte. Return true if accepted, false if dropped (full).
    fn push(&mut self, b: u8) -> bool;
    /// Mark the end of a frame. The sink may use this to atomically
    /// publish a buffered frame, or to discard it if any push during
    /// the frame failed. Default: no-op (suitable for infinite sinks).
    fn commit_frame(&mut self) {}
}

/// Encoder state.
///
/// One sample is a packed `u32` whose LE bytes are the on-wire layout
/// `[pa_lo pa_hi pb_lo pb_hi]`. Keeping the sample as a single word
/// turns the hot-path "did this sample extend the run" check into a
/// single 32-bit compare, and lets us emit samples as `to_le_bytes()`
/// without recombining halves.
pub struct Encoder {
    /// Run-in-progress: the same packed sample repeated `run_len` times.
    /// `run_len` ∈ [0, MAX_RUN]; 0 means "no run pending".
    run_sample: u32,
    run_len: u16,
    /// Block-of-unique-in-progress: up to MAX_BLOCK distinct samples
    /// that haven't been flushed yet. Stored as flat LE u32 (= the
    /// on-wire byte layout).
    block: [u8; 4 * MAX_BLOCK],
    block_n: usize,
}

impl Default for Encoder {
    fn default() -> Self {
        Self {
            run_sample: 0,
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

    /// Feed one packed sample. Layout: low 16 bits = `pa`, high 16 bits
    /// = `pb`. Callers split or pack ports as appropriate for their
    /// hardware (e.g. RP2350 PIO already produces a u32; STM32 ports
    /// are joined via `pa as u32 | (pb as u32) << 16`).
    ///
    /// May flush 0, 1, or 2 frames to `sink`.
    pub fn feed<S: Sink>(&mut self, sample: u32, sink: &mut S) {
        // If a run is in progress and this sample extends it, just bump.
        if self.run_len > 0 && sample == self.run_sample {
            self.run_len += 1;
            // Transition from 1 to 2 = a real run has appeared. The
            // protocol requires the in-progress block (samples *older*
            // than this run) to be emitted first, so the wire stream
            // stays temporally ordered.
            if self.run_len == 2 {
                self.flush_block(sink);
            }
            if self.run_len == MAX_RUN {
                self.flush_run(sink);
            }
            return;
        }

        // Not extending a run. Two sub-cases.

        // (a) We have a run of length ≥ 2 to flush. The block was already
        // drained when this run hit length 2, so we just flush the run
        // and seed a new block with the incoming sample.
        if self.run_len >= 2 {
            self.flush_run(sink);
            self.push_to_block(sample, sink);
            return;
        }

        // (b) Run of length 1 (or zero). That single sample, if it
        // exists, joins the block; then the new sample joins.
        if self.run_len == 1 {
            let prev = self.run_sample;
            self.run_len = 0;
            self.push_to_block(prev, sink);
        }

        // New sample becomes the seed of a new run-of-1.
        self.run_sample = sample;
        self.run_len = 1;
    }

    /// Push one sample into the block buffer. If the block fills, flush
    /// it as a tag=0x01 frame.
    fn push_to_block<S: Sink>(&mut self, sample: u32, sink: &mut S) {
        let off = self.block_n * 4;
        self.block[off..off + 4].copy_from_slice(&sample.to_le_bytes());
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
        sink.commit_frame();
        self.block_n = 0;
    }

    fn flush_run<S: Sink>(&mut self, sink: &mut S) {
        if self.run_len == 0 {
            return;
        }
        if self.run_len == 1 {
            // A lone run-of-1 belongs in a block, not a tag=0x02 frame.
            let s = self.run_sample;
            self.run_len = 0;
            self.push_to_block(s, sink);
            return;
        }
        // run_len >= 2 → emit as tag=0x02 (u16 count, LE).
        sink.push(TAG_RUN);
        let n = self.run_len.to_le_bytes();
        sink.push(n[0]);
        sink.push(n[1]);
        for &b in &self.run_sample.to_le_bytes() {
            sink.push(b);
        }
        sink.commit_frame();
        self.run_len = 0;
    }

    /// Flush whatever is accumulated. Call at end-of-burst or on STOP.
    pub fn flush<S: Sink>(&mut self, sink: &mut S) {
        // At this point either:
        //   - run_len ≥ 2: the block was drained on the 1→2 transition,
        //     so flush the run alone.
        //   - run_len == 1: a lone trailing sample. `flush_run` will
        //     reroute it through `push_to_block`, where it lands after
        //     any older block entries; then we flush the block.
        //   - run_len == 0: block may have unflushed entries; emit them.
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
    sink.commit_frame();
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
    sink.commit_frame();
}

/// Encode tag=0xFB STARTED ack.
pub fn encode_started<S: Sink>(sink: &mut S) {
    sink.push(TAG_STARTED);
    sink.commit_frame();
}

/// Encode tag=0xFC STOPPED ack.
pub fn encode_stopped<S: Sink>(sink: &mut S) {
    sink.push(TAG_STOPPED);
    sink.commit_frame();
}
