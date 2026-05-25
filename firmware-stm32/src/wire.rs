//! Wire protocol encoder (firmware -> host).
//!
//! See `docs/wire_protocol.md` for the full spec. This module turns a
//! stream of packed `(pa, pb)` samples into a byte sequence of
//! tag=0x01 (block-of-unique) and tag=0x02 (run) frames, with helpers
//! for tag=0xFD (overrun) and tag=0xFE (log).
//!
//! The encoder is stateful: it accumulates a pending run AND a pending
//! block-of-unique, flushing whichever rule fires first. `flush()` must
//! be called whenever streaming transitions to STOPPED or to handle
//! end-of-burst.
//!
//! This is byte-for-byte identical to `firmware/src/wire.rs` — the two
//! boards emit the same wire format so the host's parser is shared.
//! What varies per board is the *meaning* of `(pa, pb)`; the host
//! applies a board-specific permutation table to recover
//! `(data, dc, cs)`. On the F103 the natural mapping is `pa = GPIOA->IDR`
//! and `pb = GPIOB->IDR`, joined into one u32 as `pa | pb << 16`.

pub const TAG_BLOCK: u8 = 0x01;
pub const TAG_RUN: u8 = 0x02;
pub const TAG_OVERRUN: u8 = 0xFD;
pub const TAG_LOG: u8 = 0xFE;
pub const TAG_STARTED: u8 = 0xFB;
pub const TAG_STOPPED: u8 = 0xFC;

pub const HOST_CMD_START: u8 = 0x01;
pub const HOST_CMD_STOP: u8 = 0x02;
pub const HOST_CMD_STATS: u8 = 0x04;

/// Maximum samples per tag=0x02 run. u16-wide so a single RUN frame
/// can absorb up to 65535 same-color pixels — at 667 kHz peak that's
/// ~100 ms of solid fill in a single 7-byte frame.
const MAX_RUN: u16 = u16::MAX;
/// Cap unique-block frames at 16 samples instead of the protocol's
/// maximum 255. Real target traffic is almost entirely RUN frames
/// (long pixel splats); BLOCKs only carry short command/parameter
/// bursts which fit in 16 samples easily. The win is RAM: encoder
/// state and the sink's frame-staging scratch drop from ~1 KiB each
/// to ~66 B, freeing SRAM for the capture rings. The wire protocol's
/// host parser doesn't care about the upper bound.
const MAX_BLOCK: usize = 16;

/// Byte sink: where the encoder pushes wire bytes.
pub trait Sink {
    /// Push a contiguous slice of bytes. The sink may block until the
    /// whole slice is accepted; the encoder assumes it never short-
    /// writes (a torn frame would desync the host's parser).
    fn write(&mut self, buf: &[u8]);
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
    /// hardware. May flush 0, 1, or 2 frames to `sink`.
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

        // (a) We have a run of length ≥ 2 to flush. The block was
        // already drained when this run hit length 2, so just flush
        // the run. Then seed the new sample as the start of a
        // fresh run-of-1 (NOT into the block — pushing into block
        // here would cause a spurious BLOCK n=1 to emit when the
        // next sample extends the run to length 2, fragmenting a
        // continuous color into "BLOCK 1 X / RUN N X").
        if self.run_len >= 2 {
            self.flush_run(sink);
            self.run_sample = sample;
            self.run_len = 1;
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
        let bytes = self.block_n * 4;
        sink.write(&[TAG_BLOCK, self.block_n as u8]);
        sink.write(&self.block[..bytes]);
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
        // run_len >= 2 → emit as tag=0x02.
        let n = self.run_len.to_le_bytes();
        let bytes = self.run_sample.to_le_bytes();
        sink.write(&[TAG_RUN, n[0], n[1], bytes[0], bytes[1], bytes[2], bytes[3]]);
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
    let b = dropped.to_le_bytes();
    sink.write(&[TAG_OVERRUN, b[0], b[1], b[2], b[3]]);
}

/// Encode a tag=0xFE log frame. `msg` is truncated to 256 bytes.
pub fn encode_log<S: Sink>(msg: &str, sink: &mut S) {
    let bytes = msg.as_bytes();
    let len = bytes.len().min(256);
    sink.write(&[TAG_LOG, len as u8, (len >> 8) as u8]);
    sink.write(&bytes[..len]);
}

/// Encode tag=0xFB STARTED ack.
pub fn encode_started<S: Sink>(sink: &mut S) {
    sink.write(&[TAG_STARTED]);
}

/// Encode tag=0xFC STOPPED ack.
pub fn encode_stopped<S: Sink>(sink: &mut S) {
    sink.write(&[TAG_STOPPED]);
}
