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
pub const TAG_TICK: u8 = 0x03;
pub const TAG_REPEAT2: u8 = 0x04;
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

/// Maximum run length representable in a tag=0x04 REPEAT2 byte. Runs
/// longer than this force the active REPEAT2 frame closed and revert
/// to plain tag=0x02 RUN encoding for the overlong run.
const REPEAT2_MAX_LEN: u16 = 255;

/// Maximum runs we'll accumulate before force-emitting a REPEAT2
/// frame. Keeps per-frame size bounded for the sink's atomic-write
/// budget (1024-byte cap on PipeSink) and means a single long
/// alternation doesn't sit forever waiting for a flush.
const REPEAT2_BUF_RUNS: usize = 512;

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

    // ---- REPEAT2 state ----
    //
    // REPEAT2 collapses an alternating sequence `[A×la, B×lb, A×la',
    // B×lb', …]` into a single header + 1 byte per run. We accumulate
    // all run-length bytes in `r2_buf` and emit one atomic frame on
    // close — buffering keeps the PipeSink's frame-staging clean for
    // interleaved TICK/LOG/etc frames (a partial REPEAT2 sharing the
    // sink's staging would corrupt the next non-encoder frame).
    //
    // Lifecycle:
    //   0. idle → first run completes → store in `(pending_len,
    //      pending_sample)`.
    //   1. one pending run, next run has different sample and both
    //      lens ≤ 255 → seed `r2_buf` with both length bytes,
    //      switch to active.
    //   2. active, next run matches the alternation → push its
    //      length byte into `r2_buf`.
    //   3. active, next run breaks the pattern (third value, or
    //      len > 255), buffer fills, or `flush()`/`flush_block()`
    //      runs → emit one REPEAT2 frame `[tag][val_a:4][val_b:4]
    //      [r2_buf...][0]`, restart from idle.
    /// True when `r2_buf` holds in-progress run-length bytes.
    r2_active: bool,
    /// `val_a` = the run at even indices (0, 2, 4, …); `val_b` = odd.
    r2_val_a: u32,
    r2_val_b: u32,
    /// Parity of the *next* run we expect: false → val_a, true → val_b.
    r2_next_is_b: bool,
    /// Accumulated run-length bytes (length 0 terminator NOT included).
    r2_buf: [u8; REPEAT2_BUF_RUNS],
    r2_buf_n: usize,
    /// Deferred run held while we wait to see if the *next* completed
    /// run forms a REPEAT2 partnership. `pending_len == 0` means no
    /// deferred run.
    r2_pending_len: u16,
    r2_pending_sample: u32,
}

impl Default for Encoder {
    fn default() -> Self {
        Self {
            run_sample: 0,
            run_len: 0,
            block: [0; 4 * MAX_BLOCK],
            block_n: 0,
            r2_active: false,
            r2_val_a: 0,
            r2_val_b: 0,
            r2_next_is_b: false,
            r2_buf: [0; REPEAT2_BUF_RUNS],
            r2_buf_n: 0,
            r2_pending_len: 0,
            r2_pending_sample: 0,
        }
    }
}

impl Encoder {
    /// Hard-reset internal state. Used on STOP. Buffered REPEAT2
    /// content is dropped silently; the host re-syncs via TAG_STOPPED.
    pub fn reset(&mut self) {
        self.run_len = 0;
        self.block_n = 0;
        self.r2_active = false;
        self.r2_buf_n = 0;
        self.r2_next_is_b = false;
        self.r2_pending_len = 0;
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
        // drained when this run hit length 2, so we just flush the run.
        // The incoming sample seeds a fresh run-of-1 — NOT a block
        // entry — so that a subsequent same-sample stream (very
        // common right after `flush_block_only` at a drain boundary,
        // where the encoder has just emitted RUN n=K of X and the
        // next drain begins with more X) can merge into one big RUN
        // instead of splitting into BLOCK n=1 + RUN n=K-1.
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
        // Any in-flight REPEAT2 must close before another frame can be
        // emitted, otherwise the BLOCK header would land inside the
        // REPEAT2 body and corrupt the wire.
        self.repeat2_close(sink);
        sink.push(TAG_BLOCK);
        let bytes = self.block_n * 4;
        for &b in &self.block[..bytes] {
            sink.push(b);
        }
        // Sentinel 0xffff_ffff terminates the sample list — never a
        // legal sample (captures are masked to 18 bits).
        sink.push(0xff);
        sink.push(0xff);
        sink.push(0xff);
        sink.push(0xff);
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
        // run_len >= 2 → route through the REPEAT2 commit path. It
        // decides whether this run extends an in-flight REPEAT2 frame,
        // becomes the second half of a new REPEAT2 partnership, or
        // falls through to a plain tag=0x02 RUN.
        let len = self.run_len;
        let sample = self.run_sample;
        self.run_len = 0;
        self.commit_completed_run(len, sample, sink);
    }

    /// A run of `len` (≥ 2) consecutive same-sample edges has just
    /// finished. Decide between three outcomes:
    ///   - extend the active REPEAT2 buffer with one more length byte
    ///   - pair with a deferred previous run and start a new REPEAT2
    ///   - fall back to emitting a normal tag=0x02 RUN
    fn commit_completed_run<S: Sink>(&mut self, len: u16, sample: u32, sink: &mut S) {
        // If REPEAT2 is active, try to extend.
        if self.r2_active {
            let expected = if self.r2_next_is_b {
                self.r2_val_b
            } else {
                self.r2_val_a
            };
            if sample == expected && len <= REPEAT2_MAX_LEN {
                self.r2_buf[self.r2_buf_n] = len as u8;
                self.r2_buf_n += 1;
                self.r2_next_is_b = !self.r2_next_is_b;
                if self.r2_buf_n == REPEAT2_BUF_RUNS {
                    // Buffer full — emit and start fresh. The very
                    // next run can't continue the alternation across
                    // a frame boundary (we lose val_a/val_b), so we
                    // just close cleanly here.
                    self.repeat2_close(sink);
                }
                return;
            }
            // Pattern broken — close the in-flight frame, then
            // re-enter normal logic with this run as the breaker.
            self.repeat2_close(sink);
            // Fall through to deferred / direct-emit logic below.
        }

        // No active stream. If we have a deferred previous run, see if
        // (prev, current) forms a REPEAT2 partnership.
        if self.r2_pending_len > 0 {
            let prev_len = self.r2_pending_len;
            let prev_sample = self.r2_pending_sample;
            self.r2_pending_len = 0;
            if sample != prev_sample
                && prev_len <= REPEAT2_MAX_LEN
                && len <= REPEAT2_MAX_LEN
            {
                // Start a REPEAT2 frame: seed the buffer with the two
                // length bytes; header is written on emit.
                self.r2_val_a = prev_sample;
                self.r2_val_b = sample;
                self.r2_next_is_b = false; // next run should be val_a again
                self.r2_active = true;
                self.r2_buf[0] = prev_len as u8;
                self.r2_buf[1] = len as u8;
                self.r2_buf_n = 2;
                return;
            }
            // Can't partner — emit the deferred run as plain RUN. The
            // current run still wants a chance to defer, handled below.
            self.emit_plain_run(prev_len, prev_sample, sink);
            // Fall through.
        }

        // No deferred run waiting. Try to defer this one for future
        // partnering — but only if it fits the u8 budget. Overlong
        // runs always go straight to a plain RUN since they can't
        // participate in REPEAT2 anyway.
        if len <= REPEAT2_MAX_LEN {
            self.r2_pending_len = len;
            self.r2_pending_sample = sample;
        } else {
            self.emit_plain_run(len, sample, sink);
        }
    }

    /// Emit a plain tag=0x02 RUN frame.
    fn emit_plain_run<S: Sink>(&mut self, len: u16, sample: u32, sink: &mut S) {
        sink.push(TAG_RUN);
        let n = len.to_le_bytes();
        sink.push(n[0]);
        sink.push(n[1]);
        for &b in &sample.to_le_bytes() {
            sink.push(b);
        }
        sink.commit_frame();
    }

    /// Emit any buffered REPEAT2 content as one atomic frame, and
    /// drain any deferred-pending run as a plain RUN. Idempotent.
    fn repeat2_close<S: Sink>(&mut self, sink: &mut S) {
        if self.r2_active {
            sink.push(TAG_REPEAT2);
            for &b in &self.r2_val_a.to_le_bytes() {
                sink.push(b);
            }
            for &b in &self.r2_val_b.to_le_bytes() {
                sink.push(b);
            }
            for &b in &self.r2_buf[..self.r2_buf_n] {
                sink.push(b);
            }
            sink.push(0u8);
            sink.commit_frame();
            self.r2_active = false;
            self.r2_buf_n = 0;
            self.r2_next_is_b = false;
        }
        if self.r2_pending_len > 0 {
            let len = self.r2_pending_len;
            let sample = self.r2_pending_sample;
            self.r2_pending_len = 0;
            self.emit_plain_run(len, sample, sink);
        }
    }

    /// Flush whatever is accumulated. Call at end-of-burst or on STOP.
    pub fn flush<S: Sink>(&mut self, sink: &mut S) {
        // Temporal order at flush time, oldest → newest:
        //   1. Any REPEAT2 stream content / deferred-pending run (these
        //      were committed when older samples differed from the
        //      current run-in-progress).
        //   2. Block content (lone run-of-1 samples *after* the last
        //      committed run, demoted into the block).
        //   3. The current run-in-progress (run_len > 0).
        //
        // `flush_run` handles case 3, routing run_len ≥ 2 through
        // `commit_completed_run` (which may extend REPEAT2 stream — so
        // we close that *after*) and run_len == 1 through the block.
        self.flush_run(sink);
        // Close any active/deferred REPEAT2 BEFORE block so RUNs that
        // emerge from r2 emission stay temporally older than block.
        self.repeat2_close(sink);
        self.flush_block(sink);
    }

    /// Flush only the in-progress block, leaving any pending run alive
    /// so it can continue to extend on the next call. Use this at
    /// drain-boundary points where you want to keep latency bounded
    /// for unique-sample bursts (BLOCKs don't compress further by
    /// waiting) but still let a steady same-sample stream merge across
    /// drains into a single big RUN frame. Also leaves any in-flight
    /// REPEAT2 stream alive — alternating pixel bursts very commonly
    /// span many drain boundaries and would lose almost all their
    /// compression if forced to flush each tick.
    pub fn flush_block_only<S: Sink>(&mut self, sink: &mut S) {
        self.flush_block(sink);
    }
}

/// Encode a tag=0x03 drain-tick frame.
pub fn encode_tick<S: Sink>(
    t_us: u32,
    dt_us: u16,
    n_drained: u16,
    n_pending: u16,
    bytes_out: u32,
    sink: &mut S,
) {
    sink.push(TAG_TICK);
    for &b in &t_us.to_le_bytes() {
        sink.push(b);
    }
    for &b in &dt_us.to_le_bytes() {
        sink.push(b);
    }
    for &b in &n_drained.to_le_bytes() {
        sink.push(b);
    }
    for &b in &n_pending.to_le_bytes() {
        sink.push(b);
    }
    for &b in &bytes_out.to_le_bytes() {
        sink.push(b);
    }
    sink.commit_frame();
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
