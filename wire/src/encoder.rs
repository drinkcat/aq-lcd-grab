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
/// A sequence of runs strictly alternating between two sample values.
pub const TAG_REPEAT2: u8 = 0x03;

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

/// Largest run length representable as a REPEAT2 length byte (u8). A
/// run longer than this can't join a REPEAT2 frame and falls back to a
/// plain RUN.
const MAX_REPEAT2_LEN: u16 = 255;

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
    /// A REPEAT2 frame is open: its tag + `val_a`/`val_b` header and one
    /// or more length bytes have been pushed but its `0x00` terminator
    /// has not. Runs alternate `val_a, val_b, val_a, …`; `next_is_b`
    /// gives the value the next run must match to extend the frame.
    Repeat2Open {
        val_a: u32,
        val_b: u32,
        next_is_b: bool,
    },
}

/// One pending run: `len` consecutive identical `sample`s.
#[derive(Clone, Copy, Default)]
struct Run {
    sample: u32,
    len: u16,
}

impl Run {
    fn new(sample: u32, len: u16) -> Self {
        Self { sample, len }
    }
}

/// Streaming encoder over packed `u32` samples.
#[derive(Default)]
pub struct Encoder {
    /// The run currently being extended by `feed`. `len == 0` means no
    /// run in progress.
    run: Run,
    /// Completed runs held back to test for a REPEAT2 alternation, in
    /// temporal order (`deferred[0]` oldest). A REPEAT2 opens only once
    /// a third run *confirms* the A/B pattern (`A, B, A`), so we hold up
    /// to two candidates here while `state` is Idle/BlockOpen. Once a
    /// REPEAT2 is open, completed runs extend it directly and this is
    /// empty.
    deferred: [Run; 2],
    n_def: usize,
    /// Which frame is mid-emission on the sink.
    state: State,
}

impl Encoder {
    /// Drop all pending state. Call on STOP; the host re-syncs.
    ///
    /// This does *not* close an open frame on the wire — it forgets that
    /// one was open. Only call when the stream is being abandoned (the
    /// host discards bytes until the STOPPED ack).
    pub fn reset(&mut self) {
        self.run = Run::default();
        self.n_def = 0;
        self.state = State::Idle;
    }

    /// Feed one packed sample.
    pub fn feed<S: Sink>(&mut self, sample: u32, sink: &mut S) {
        // Same sample as the run in progress: just extend it.
        if self.run.len > 0 && sample == self.run.sample {
            self.run.len += 1;
            // A RUN frame caps at MAX_RUN edges; complete the run so it
            // is emitted (it can't grow into a single longer frame).
            if self.run.len == MAX_RUN {
                self.complete_run(sink);
            }
            return;
        }

        // Different sample: the run in progress is complete. Resolve it,
        // then open a fresh run-of-1 with the incoming sample.
        self.complete_run(sink);
        self.run = Run::new(sample, 1);
    }

    /// Flush all pending state. Call at end-of-burst or on STOP.
    pub fn flush<S: Sink>(&mut self, sink: &mut S) {
        self.complete_run(sink);
        self.drain_deferred(sink);
        self.go_idle(sink);
        sink.flush();
    }

    /// The run in progress is complete. Either extend an open REPEAT2
    /// with it, or feed it into the candidate window — opening a REPEAT2
    /// as soon as three runs confirm an `A, B, A` alternation. Until
    /// then candidates are held; an unconfirmable oldest run is resolved
    /// as a plain RUN / BLOCK sample and the window slides. Clears `run`.
    fn complete_run<S: Sink>(&mut self, sink: &mut S) {
        let incoming = self.run;
        if incoming.len == 0 {
            return;
        }
        self.run = Run::default();

        // Extend an open REPEAT2 if this run matches the alternation and
        // fits a u8 length byte (len 1..=255 — a lone sample participates
        // like any other run).
        if let State::Repeat2Open { val_a, val_b, next_is_b } = self.state {
            let expect = if next_is_b { val_b } else { val_a };
            if incoming.sample == expect && incoming.len <= MAX_REPEAT2_LEN {
                sink.push(incoming.len as u8);
                self.state = State::Repeat2Open { val_a, val_b, next_is_b: !next_is_b };
                return;
            }
            // Pattern broken (third value or overlong): close the frame
            // and fall through to the candidate window (now empty).
            self.go_idle(sink);
        }

        // Feed `incoming` into the candidate window.

        // A run too long for a u8 length byte can never be in a REPEAT2:
        // drain the window and emit it as a plain RUN.
        if incoming.len > MAX_REPEAT2_LEN {
            self.drain_deferred(sink);
            self.go_idle(sink);
            self.emit_run(incoming.len, incoming.sample, sink);
            return;
        }

        if self.n_def < 2 {
            self.deferred[self.n_def] = incoming;
            self.n_def += 1;
            return;
        }

        // Window is [a, b]; `incoming` is the third run c.
        let a = self.deferred[0];
        let b = self.deferred[1];
        let c = incoming;
        if a.sample == c.sample {
            // Confirmed alternation a, b, a → open the frame.
            self.go_idle(sink); // close any open BLOCK
            sink.push(TAG_REPEAT2);
            sink.push_u32(a.sample);
            sink.push_u32(b.sample);
            sink.push(a.len as u8);
            sink.push(b.len as u8);
            sink.push(c.len as u8);
            self.n_def = 0;
            self.state = State::Repeat2Open {
                val_a: a.sample,
                val_b: b.sample,
                next_is_b: true, // a,b,a emitted; next run should be b
            };
            return;
        }
        // Not confirmed: `a` can't start a REPEAT2. Resolve it and slide
        // the window down — `b` and `c` become the new candidate pair.
        // (`c` fits a u8: the overlong check above already ruled it out.)
        self.resolve_single(a, sink);
        self.deferred[0] = b;
        self.deferred[1] = c;
        self.n_def = 2;
    }

    /// Emit a single unconfirmed run on its own: a lone sample joins the
    /// BLOCK, a run of ≥ 2 becomes a plain RUN.
    fn resolve_single<S: Sink>(&mut self, run: Run, sink: &mut S) {
        if run.len == 1 {
            self.append_block(run.sample, sink);
        } else {
            self.go_idle(sink);
            self.emit_run(run.len, run.sample, sink);
        }
    }

    /// Resolve every deferred candidate (oldest first) on its own. Used
    /// at flush and whenever the candidate window must be cleared before
    /// emitting another frame.
    fn drain_deferred<S: Sink>(&mut self, sink: &mut S) {
        for i in 0..self.n_def {
            self.resolve_single(self.deferred[i], sink);
        }
        self.n_def = 0;
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
            State::Repeat2Open { .. } => {
                self.go_idle(sink);
                sink.push(TAG_BLOCK);
                self.state = State::BlockOpen;
            }
        }
        sink.push_u32(sample);
    }

    /// Terminate whatever frame is open and return to `Idle`. BLOCK
    /// writes the sentinel; REPEAT2 writes the `0x00` terminator. No-op
    /// when already idle.
    fn go_idle<S: Sink>(&mut self, sink: &mut S) {
        match self.state {
            State::Idle => {}
            State::BlockOpen => {
                sink.push_u32(BLOCK_SENTINEL);
                self.state = State::Idle;
            }
            State::Repeat2Open { .. } => {
                sink.push(0);
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
    ///
    /// Unlike the telemetry frames, OVERRUN marks a *position* in the
    /// data stream — the gap is here. So it first flushes all pending
    /// captured data (the in-progress run and any deferred candidates),
    /// otherwise that data would land after the gap marker and the host
    /// would mis-place the gap.
    pub fn overrun<S: Sink>(&mut self, dropped: u32, sink: &mut S) {
        self.complete_run(sink);
        self.drain_deferred(sink);
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

    /// `run_lens` are the lengths of runs alternating val_a, val_b, …
    fn repeat2_frame(val_a: u32, val_b: u32, run_lens: &[u8]) -> Vec<u8> {
        let mut v = vec![TAG_REPEAT2];
        v.extend_from_slice(&val_a.to_le_bytes());
        v.extend_from_slice(&val_b.to_le_bytes());
        v.extend_from_slice(run_lens);
        v.push(0);
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
    fn two_runs_alone_do_not_open_repeat2() {
        // 7×2 then 9×3 — only two runs, no third to confirm the A/B
        // alternation, so they stay as two plain RUNs (not a 2-run
        // REPEAT2, which would be larger).
        assert_eq!(
            encode(&[7, 7, 9, 9, 9]),
            frames(&[run_frame(2, 7), run_frame(3, 9)]),
        );
    }

    #[test]
    fn three_runs_confirm_and_open_repeat2() {
        // A×2 B×3 A×2 — the third run (A) confirms the alternation and
        // opens the frame with lens [2,3,2].
        assert_eq!(encode(&[7, 7, 9, 9, 9, 7, 7]), repeat2_frame(7, 9, &[2, 3, 2]));
    }

    #[test]
    fn alternating_runs_extend_one_repeat2() {
        // A×2 B×2 A×3 B×4 A×2 — strict alternation, one frame.
        let s = [1, 1, 2, 2, 1, 1, 1, 2, 2, 2, 2, 1, 1];
        assert_eq!(encode(&s), repeat2_frame(1, 2, &[2, 2, 3, 4, 2]));
    }

    #[test]
    fn len_one_runs_participate_in_repeat2() {
        // 7×3 1×1 7×3 1×1 7×5 — a thin pattern where one side is a lone
        // sample. The third run (7) confirms, and len-1 runs extend the
        // frame: lens [3,1,3,1,5].
        let s = [7, 7, 7, 1, 7, 7, 7, 1, 7, 7, 7, 7, 7];
        assert_eq!(encode(&s), repeat2_frame(7, 1, &[3, 1, 3, 1, 5]));
    }

    #[test]
    fn distinct_command_bytes_stay_a_block() {
        // Four distinct single samples with no A/B/A alternation. None
        // can confirm a REPEAT2, so they stay one compact BLOCK rather
        // than a string of wasteful tiny REPEAT2 frames.
        let s = [0x2a, 0x00, 0xa9, 0xb8];
        assert_eq!(encode(&s), block_frame(&s));
    }

    #[test]
    fn third_value_breaks_then_new_pair_needs_its_own_third() {
        // A×2 B×2 A×2 (confirms -> REPEAT2 lens [2,2,2]) then C×2 D×2 —
        // only two runs after the break, so they stay as plain RUNs.
        let s = [1, 1, 2, 2, 1, 1, 3, 3, 4, 4];
        assert_eq!(
            encode(&s),
            frames(&[
                repeat2_frame(1, 2, &[2, 2, 2]),
                run_frame(2, 3),
                run_frame(2, 4),
            ]),
        );
    }

    #[test]
    fn two_run_pair_with_no_third_stays_two_runs() {
        // A×2 B×2 then end — no confirming third run, two plain RUNs.
        assert_eq!(
            encode(&[5, 5, 8, 8]),
            frames(&[run_frame(2, 5), run_frame(2, 8)]),
        );
    }

    #[test]
    fn single_run_with_no_partner_is_a_plain_run() {
        // Only one run ever completes -> deferred, then flushed as RUN.
        assert_eq!(encode(&[5, 5]), run_frame(2, 5));
    }

    #[test]
    fn overlong_run_cannot_join_repeat2() {
        // First run fits a u8 (200), second exceeds 255 -> no partnership;
        // the 200-run flushes as a plain RUN, the long run splits.
        let n = 300usize;
        let mut s = vec![7u32; 200];
        s.extend(std::iter::repeat(9).take(n));
        // 7×200 deferred; 9×300 completes but >255 so can't partner:
        // drain deferred (RUN 200,7), then 9-run emitted as plain RUN(s).
        // 9×300 was split at MAX_RUN? No — 300 < MAX_RUN, so one RUN(300,9).
        assert_eq!(
            encode(&s),
            frames(&[run_frame(200, 7), run_frame(300, 9)]),
        );
    }

    #[test]
    fn overlong_third_run_prevents_repeat2() {
        // A×2 B×2 A×256 — the third run matches A but is too long for a
        // u8 length byte, so it can't confirm a REPEAT2. The deferred A,B
        // drain as plain RUNs and the long run follows as its own RUN.
        let mut s = vec![1u32, 1, 2, 2];
        s.extend(std::iter::repeat(1).take(256));
        assert_eq!(
            encode(&s),
            frames(&[run_frame(2, 1), run_frame(2, 2), run_frame(256, 1)]),
        );
    }

    #[test]
    fn block_then_single_run_keeps_order() {
        // lone 9 (BLOCK), then 5×2 with no partner — the BLOCK must
        // close before the RUN, preserving temporal order.
        assert_eq!(
            encode(&[9, 5, 5]),
            frames(&[block_frame(&[9]), run_frame(2, 5)]),
        );
    }

    #[test]
    fn block_then_repeat2() {
        // lone 9, then a confirmed alternation 1×2 2×2 1×2 — the 9 goes
        // to a BLOCK that closes before the REPEAT2 opens.
        assert_eq!(
            encode(&[9, 1, 1, 2, 2, 1, 1]),
            frames(&[block_frame(&[9]), repeat2_frame(1, 2, &[2, 2, 2])]),
        );
    }

    #[test]
    fn repeat2_then_lone_sample_becomes_block() {
        // 1×2 2×2 1×2 then lone 9 — REPEAT2 closes, 9 goes in a BLOCK.
        assert_eq!(
            encode(&[1, 1, 2, 2, 1, 1, 9]),
            frames(&[repeat2_frame(1, 2, &[2, 2, 2]), block_frame(&[9])]),
        );
    }

    #[test]
    fn run_lone_run_confirms_repeat2() {
        // 5×2 9×1 5×2 — three runs where the middle is a lone sample;
        // the third (5) confirms the alternation, so this becomes one
        // REPEAT2 with a len-1 middle run.
        assert_eq!(encode(&[5, 5, 9, 5, 5]), repeat2_frame(5, 9, &[2, 1, 2]));
    }

    #[test]
    fn lone_sample_can_be_repeat2_val_a() {
        // 9×1 5×2 9×1 — the *first* run is a lone sample. It is still
        // deferred (not dumped into a BLOCK), so the third run (9)
        // confirms an alternation and it becomes val_a of a REPEAT2.
        assert_eq!(encode(&[9, 5, 5, 9]), repeat2_frame(9, 5, &[1, 2, 1]));
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
    fn overrun_flushes_pending_data_before_the_gap() {
        // Two lone samples are pending (one deferred, one in progress).
        // OVERRUN marks a data gap, so it must emit all that captured
        // data — as a closed BLOCK — *before* the gap marker, not after.
        let mut enc = Encoder::default();
        let mut sink = VecSink::new();
        enc.feed(1, &mut sink);
        enc.feed(2, &mut sink);
        enc.overrun(0x42, &mut sink);
        enc.flush(&mut sink);
        let mut want = block_frame(&[1, 2]);
        want.extend_from_slice(&[TAG_OVERRUN, 0x42, 0, 0, 0]);
        assert_eq!(sink.bytes, want);
    }

    #[test]
    fn tick_leaves_a_pending_run_alive() {
        // A RUN in progress is held in encoder state, not on the sink.
        // TICK is position-independent telemetry, so it must NOT flush
        // the run — it keeps accumulating and emits as one frame at the
        // end (preserving cross-drain run merging).
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
