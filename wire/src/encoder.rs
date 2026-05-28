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
        for &b in &sample.to_le_bytes() {
            sink.push(b);
        }
    }

    /// Terminate whatever frame is open and return to `Idle`. For a
    /// BLOCK that means writing the sentinel. No-op when already idle.
    fn go_idle<S: Sink>(&mut self, sink: &mut S) {
        match self.state {
            State::Idle => {}
            State::BlockOpen => {
                for &b in &BLOCK_SENTINEL.to_le_bytes() {
                    sink.push(b);
                }
                self.state = State::Idle;
            }
        }
    }

    fn emit_run<S: Sink>(&mut self, len: u16, sample: u32, sink: &mut S) {
        sink.push(TAG_RUN);
        for &b in &len.to_le_bytes() {
            sink.push(b);
        }
        for &b in &sample.to_le_bytes() {
            sink.push(b);
        }
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
}
