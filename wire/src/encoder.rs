//! Sample-stream encoder.
//!
//! Turns a stream of packed `(pa, pb)` samples into RUN and BLOCK
//! frames. One sample is a packed `u32` whose little-endian bytes are
//! the on-wire layout `[pa_lo pa_hi pb_lo pb_hi]`; callers pack their
//! ports as appropriate (the Pico's PIO already yields a `u32`, the
//! STM32 joins two IDR reads as `pa | pb << 16`).
//!
//! Two frame kinds:
//!   - RUN (tag 0x02): `n` (≥ 2) consecutive identical samples.
//!   - BLOCK (tag 0x01): a span of samples where no two adjacent
//!     samples are equal, terminated by the `0xffff_ffff` sentinel.
//!
//! The encoder holds only the current run `(sample, len)` and which
//! frame is open on the sink. Each new sample either extends the run or
//! resolves it: a lone sample (len 1) joins the open BLOCK; a real run
//! (len ≥ 2) ends the BLOCK and emits a RUN. BLOCK frames are
//! sentinel-terminated with no leading count, so lone samples stream
//! straight to the sink — no per-frame buffer is needed.

use crate::Sink;

/// A span of distinct adjacent samples, amortising the frame header.
pub const TAG_BLOCK: u8 = 0x01;
/// `n` consecutive identical samples.
pub const TAG_RUN: u8 = 0x02;

// Side-band (control) frames. All live in the 0xFX range, away from the
// low data tags, and are emitted via the `Encoder` methods so any open
// BLOCK is closed first — a tag landing mid-BLOCK would corrupt the wire.
/// Drain-tick telemetry heartbeat.
pub const TAG_TICK: u8 = 0xFA;
/// STARTED acknowledgement.
pub const TAG_STARTED: u8 = 0xFB;
/// STOPPED acknowledgement.
pub const TAG_STOPPED: u8 = 0xFC;
/// Dropped-sample (capture gap) marker.
pub const TAG_OVERRUN: u8 = 0xFD;
/// Out-of-band UTF-8 log line.
pub const TAG_LOG: u8 = 0xFE;

/// Largest run length a single RUN frame can carry. Longer runs split
/// into back-to-back RUN frames.
const MAX_RUN: u16 = u16::MAX;

/// Samples are masked to 18 bits on both boards, so `0xffff_ffff` is
/// never a legal sample and can terminate the BLOCK sample list.
const BLOCK_SENTINEL: u32 = 0xffff_ffff;

/// What frame, if any, is currently open on the sink — bytes pushed but
/// not yet terminated/flushed.
#[derive(Default)]
enum State {
    /// No frame open; the next sample starts one.
    #[default]
    Idle,
    /// A BLOCK frame is open: its tag and zero or more samples have been
    /// pushed but its sentinel has not. The next lone sample appends to
    /// it; a run or `flush` closes it.
    BlockOpen,
}

/// Streaming encoder over packed `u32` samples.
#[derive(Default)]
pub struct Encoder {
    /// The sample currently being run-length counted. Valid only when
    /// `run_len > 0`.
    run_sample: u32,
    /// Length of the run in progress; 0 means no run pending.
    run_len: u16,
    /// Which frame is mid-emission on the sink.
    state: State,
}

impl Encoder {
    /// Drop all pending state. Call on STOP; the host re-syncs.
    ///
    /// This does *not* close an open BLOCK on the wire — it forgets that
    /// one was open. Only call when the stream is being abandoned (the
    /// host discards bytes until the STOPPED ack).
    pub fn reset(&mut self) {
        self.run_len = 0;
        self.state = State::Idle;
    }

    /// Feed one packed sample.
    pub fn feed<S: Sink>(&mut self, sample: u32, sink: &mut S) {
        // Same sample as the run in progress: just extend it.
        if self.run_len > 0 && sample == self.run_sample {
            self.run_len += 1;
            // A RUN frame caps at MAX_RUN edges; emit and start over.
            if self.run_len == MAX_RUN {
                self.emit_run(self.run_len, self.run_sample, sink);
                self.run_len = 0;
            }
            return;
        }

        // Different sample: the pending run is done. Resolve it, then
        // start a fresh run-of-1 with the incoming sample.
        self.resolve(sink);
        self.run_sample = sample;
        self.run_len = 1;
    }

    /// Flush all pending state. Call at end-of-burst or on STOP.
    pub fn flush<S: Sink>(&mut self, sink: &mut S) {
        self.resolve(sink);
        self.go_idle(sink);
        sink.flush();
    }

    /// Resolve the pending run: a lone sample (len 1) joins the open
    /// BLOCK; a real run (len ≥ 2) returns to idle (closing any open
    /// BLOCK), then emits a RUN. Clears `run_len`. No-op when nothing is
    /// pending.
    fn resolve<S: Sink>(&mut self, sink: &mut S) {
        match self.run_len {
            0 => {}
            1 => self.append_block(self.run_sample, sink),
            len => {
                self.go_idle(sink);
                self.emit_run(len, self.run_sample, sink);
            }
        }
        self.run_len = 0;
    }

    /// Append one lone sample to the BLOCK, starting the frame (tag) if
    /// the BLOCK isn't open yet.
    fn append_block<S: Sink>(&mut self, sample: u32, sink: &mut S) {
        match self.state {
            State::Idle => {
                sink.push(TAG_BLOCK);
                self.state = State::BlockOpen;
            }
            State::BlockOpen => {}
        }
        sink.push_u32(sample);
    }

    /// Terminate whatever frame is open and return to `Idle`. For a
    /// BLOCK that means writing the sentinel. No-op when already idle.
    fn go_idle<S: Sink>(&mut self, sink: &mut S) {
        match self.state {
            State::Idle => {}
            State::BlockOpen => {
                sink.push_u32(BLOCK_SENTINEL);
                self.state = State::Idle;
            }
        }
    }

    fn emit_run<S: Sink>(&mut self, len: u16, sample: u32, sink: &mut S) {
        sink.push(TAG_RUN);
        sink.push_u16(len);
        sink.push_u32(sample);
    }

    // ---- Side-band (control) frames ----
    //
    // Each closes any open BLOCK first (`go_idle`) so its tag never
    // lands inside a BLOCK's sample list, then writes its own frame and
    // flushes it to the wire. A pending RUN is *not* on the sink yet, so
    // it is left alive — a uniform fill spanning many drains still
    // merges into one big RUN.

    /// Emit a STARTED ack (`[0xFB]`) — firmware acknowledged START.
    pub fn started<S: Sink>(&mut self, sink: &mut S) {
        self.go_idle(sink);
        sink.push(TAG_STARTED);
        sink.flush();
    }

    /// Emit a STOPPED ack (`[0xFC]`) — firmware acknowledged STOP.
    pub fn stopped<S: Sink>(&mut self, sink: &mut S) {
        self.go_idle(sink);
        sink.push(TAG_STOPPED);
        sink.flush();
    }

    /// Emit a LOG frame: `[0xFE][utf8 bytes][0x00]`. The payload is
    /// NUL-terminated rather than length-prefixed so it can stream
    /// straight to the wire. A finite sink drops the overflow, bounding
    /// the frame on its own.
    ///
    /// `msg` must not contain an embedded NUL (`0x00`) — a NUL would be
    /// read as the terminator and the host would parse the remainder as
    /// a new frame, desyncing the stream. Firmware log text (banners,
    /// stats) never contains a NUL, so this is left to the caller.
    pub fn log<S: Sink>(&mut self, msg: &str, sink: &mut S) {
        self.go_idle(sink);
        sink.push(TAG_LOG);
        sink.push_bytes(msg.as_bytes());
        sink.push(0);
        sink.flush();
    }

    /// Emit an OVERRUN marker: `[0xFD][dropped:u32]`. `dropped` is the
    /// number of WR edges (samples) lost since the last overrun frame.
    pub fn overrun<S: Sink>(&mut self, dropped: u32, sink: &mut S) {
        self.go_idle(sink);
        sink.push(TAG_OVERRUN);
        sink.push_u32(dropped);
        sink.flush();
    }

    /// Emit a drain-TICK frame:
    /// `[0xFA][t_us:u32][dt_us:u16][n_drained:u16][n_pending:u16][bytes_out:u32]`.
    ///
    /// - `t_us`: firmware clock at the start of the drain pass (µs, wraps).
    /// - `dt_us`: wall-clock duration of the drain pass.
    /// - `n_drained`: samples consumed this pass.
    /// - `n_pending`: samples still in the capture ring after draining.
    /// - `bytes_out`: bytes enqueued this window, excluding TICK frames.
    pub fn tick<S: Sink>(
        &mut self,
        t_us: u32,
        dt_us: u16,
        n_drained: u16,
        n_pending: u16,
        bytes_out: u32,
        sink: &mut S,
    ) {
        self.go_idle(sink);
        sink.push(TAG_TICK);
        sink.push_u32(t_us);
        sink.push_u16(dt_us);
        sink.push_u16(n_drained);
        sink.push_u16(n_pending);
        sink.push_u32(bytes_out);
        sink.flush();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sink::VecSink;

    /// Feed a slice of samples through the encoder, flush, and return
    /// the flat wire bytes.
    fn encode(samples: &[u32]) -> Vec<u8> {
        let mut enc = Encoder::default();
        let mut sink = VecSink::new();
        for &s in samples {
            enc.feed(s, &mut sink);
        }
        enc.flush(&mut sink);
        sink.bytes
    }

    /// Concatenate expected frames into one flat byte stream.
    fn frames(frames: &[Vec<u8>]) -> Vec<u8> {
        frames.concat()
    }

    fn run_frame(n: u16, sample: u32) -> Vec<u8> {
        let mut v = vec![TAG_RUN];
        v.extend_from_slice(&n.to_le_bytes());
        v.extend_from_slice(&sample.to_le_bytes());
        v
    }

    fn block_frame(samples: &[u32]) -> Vec<u8> {
        let mut v = vec![TAG_BLOCK];
        for &s in samples {
            v.extend_from_slice(&s.to_le_bytes());
        }
        v.extend_from_slice(&BLOCK_SENTINEL.to_le_bytes());
        v
    }

    #[test]
    fn empty_emits_nothing() {
        assert!(encode(&[]).is_empty());
    }

    #[test]
    fn single_sample_is_a_block() {
        assert_eq!(encode(&[0xAB]), block_frame(&[0xAB]));
    }

    #[test]
    fn two_identical_samples_make_a_run() {
        assert_eq!(encode(&[0x55, 0x55]), run_frame(2, 0x55));
    }

    #[test]
    fn distinct_samples_pack_into_one_block() {
        let s = [1u32, 2, 3, 4];
        assert_eq!(encode(&s), block_frame(&s));
    }

    #[test]
    fn run_in_the_middle_splits_block_then_run_then_block() {
        // unique 1,2 | run of 3×3 | unique 4
        assert_eq!(
            encode(&[1, 2, 3, 3, 3, 4]),
            frames(&[block_frame(&[1, 2]), run_frame(3, 3), block_frame(&[4])]),
        );
    }

    #[test]
    fn adjacent_runs_of_different_samples() {
        assert_eq!(
            encode(&[7, 7, 9, 9, 9]),
            frames(&[run_frame(2, 7), run_frame(3, 9)]),
        );
    }

    #[test]
    fn run_resumes_after_a_breaking_sample() {
        // A run, broken by one distinct sample, then the same value
        // again must form a second run — not fold the lone sample and
        // the resumed value into a confused block.
        assert_eq!(
            encode(&[5, 5, 9, 5, 5]),
            frames(&[run_frame(2, 5), block_frame(&[9]), run_frame(2, 5)]),
        );
    }

    #[test]
    fn run_longer_than_u16_splits_into_multiple_run_frames() {
        let n = MAX_RUN as usize + 10;
        // First a full MAX_RUN frame, then the remainder.
        assert_eq!(
            encode(&vec![0xC; n]),
            frames(&[run_frame(MAX_RUN, 0xC), run_frame(10, 0xC)]),
        );
    }

    // ---- side-band signals ----

    /// Run a side-band method on a fresh encoder/sink and return bytes.
    fn signal(f: impl FnOnce(&mut Encoder, &mut VecSink)) -> Vec<u8> {
        let mut enc = Encoder::default();
        let mut sink = VecSink::new();
        f(&mut enc, &mut sink);
        sink.bytes
    }

    #[test]
    fn started_is_one_byte() {
        assert_eq!(signal(|e, s| e.started(s)), vec![TAG_STARTED]);
    }

    #[test]
    fn stopped_is_one_byte() {
        assert_eq!(signal(|e, s| e.stopped(s)), vec![TAG_STOPPED]);
    }

    #[test]
    fn log_frames_text_nul_terminated() {
        assert_eq!(
            signal(|e, s| e.log("hi", s)),
            vec![TAG_LOG, b'h', b'i', 0],
        );
    }

    #[test]
    fn log_passes_full_payload_through() {
        let msg = "x".repeat(500);
        let out = signal(|e, s| e.log(&msg, s));
        assert_eq!(out.len(), 1 + 500 + 1); // tag + payload + NUL
        assert_eq!(out[0], TAG_LOG);
        assert_eq!(*out.last().unwrap(), 0);
    }

    #[test]
    fn overrun_carries_u32_le() {
        assert_eq!(
            signal(|e, s| e.overrun(0x0A0B0C0D, s)),
            vec![TAG_OVERRUN, 0x0D, 0x0C, 0x0B, 0x0A],
        );
    }

    #[test]
    fn tick_field_order_is_le() {
        let out = signal(|e, s| e.tick(0x11223344, 0x5566, 0x7788, 0x99AA, 0xBBCCDDEE, s));
        let mut want = vec![TAG_TICK];
        want.extend_from_slice(&0x11223344u32.to_le_bytes());
        want.extend_from_slice(&0x5566u16.to_le_bytes());
        want.extend_from_slice(&0x7788u16.to_le_bytes());
        want.extend_from_slice(&0x99AAu16.to_le_bytes());
        want.extend_from_slice(&0xBBCCDDEEu32.to_le_bytes());
        assert_eq!(out, want);
    }

    #[test]
    fn signal_closes_an_open_block_first() {
        // A lone sample opens a BLOCK; emitting a side-band frame
        // mid-stream must terminate the BLOCK (sentinel) before the
        // side-band tag, or the 0xFD would be parsed as a BLOCK sample.
        // (feed(2) leaves sample 1 in the open BLOCK and sample 2 as a
        // not-yet-emitted pending run, so only `1` is on the sink.)
        let mut enc = Encoder::default();
        let mut sink = VecSink::new();
        enc.feed(1, &mut sink);
        enc.feed(2, &mut sink);
        enc.overrun(0x42, &mut sink);
        enc.flush(&mut sink);
        let mut want = block_frame(&[1]);
        want.extend_from_slice(&[TAG_OVERRUN, 0x42, 0, 0, 0]);
        want.extend_from_slice(&block_frame(&[2]));
        assert_eq!(sink.bytes, want);
    }

    #[test]
    fn signal_leaves_a_pending_run_alive() {
        // A RUN in progress is held in encoder state, not on the sink,
        // so a side-band frame must NOT flush it — the run keeps
        // accumulating and emits as one frame at the end.
        let mut enc = Encoder::default();
        let mut sink = VecSink::new();
        enc.feed(7, &mut sink);
        enc.feed(7, &mut sink); // run of 2 pending, nothing on sink yet
        enc.tick(0, 0, 0, 0, 0, &mut sink);
        enc.feed(7, &mut sink); // extends to 3
        enc.flush(&mut sink);
        let mut want = vec![TAG_TICK];
        want.extend_from_slice(&[0; 14]); // all-zero tick body
        want.extend_from_slice(&run_frame(3, 7));
        assert_eq!(sink.bytes, want);
    }
}
