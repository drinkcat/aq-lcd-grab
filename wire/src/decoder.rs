//! Streaming, allocation-free wire-format decoder.
//!
//! Mirrors [`crate::Encoder`]; see `docs/wire_protocol.md` for the framing
//! spec. Feed raw bytes in via [`Decoder::feed`] and receive decoded
//! [`WireEvent`]s through a callback. The decoder keeps any partial trailing
//! frame in a fixed internal buffer for the next call, so the caller can pass
//! whatever-size reads the transport returns.
//!
//! Variable-length payloads (`Block` samples, `Repeat2` run lengths) are
//! handed to the callback as **borrowed slices** into the decoder's internal
//! buffer — no `Vec`, no `String`, no allocation. A frame larger than the
//! internal buffers is split across multiple events (e.g. a long `Block` is
//! emitted in chunks); since consumers process samples element-by-element this
//! is transparent to the decoded sample stream.
//!
//! Frame integrity is the firmware's job (it stages frames atomically into its
//! TX pipe). An unrecognised leading tag means the stream desynced —
//! [`Decoder::feed`] returns [`WireError::BadTag`] and the caller should resync
//! with STOP/drain/START.

pub const TAG_BLOCK: u8 = 0x01;
pub const TAG_RUN: u8 = 0x02;
pub const TAG_REPEAT2: u8 = 0x03;
pub const TAG_TICK: u8 = 0xFA;
pub const TAG_STARTED: u8 = 0xFB;
pub const TAG_STOPPED: u8 = 0xFC;
pub const TAG_OVERRUN: u8 = 0xFD;
pub const TAG_LOG: u8 = 0xFE;

const BLOCK_SENTINEL: u32 = 0xffff_ffff;

/// Capacity (in bytes) of the buffer that holds the partial trailing frame
/// across `feed` calls, and the body of variable-length frames before they are
/// emitted. A `Block` or `Repeat2` body longer than this is emitted in chunks.
/// 1 KiB comfortably holds the largest atomic frame the firmware stages while
/// staying tiny on the ESP32.
const BUF_CAP: usize = 1024;

/// One decoded event. Variable-length payloads borrow from the decoder's
/// internal buffer and are only valid for the duration of the callback.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WireEvent<'a> {
    /// A tag=0x01 block, expanded into raw samples. A long block may be
    /// delivered as several consecutive `Block` events.
    Block(&'a [u32]),
    /// A tag=0x02 run: `n` repetitions of `sample`.
    Run { n: u16, sample: u32 },
    /// A tag=0x03 REPEAT2: a sequence of runs alternating between `val_a` and
    /// `val_b`. `run_lens[i]` is the length of run `i`; run 0 is `val_a`, run 1
    /// is `val_b`, run 2 is `val_a`, etc. Each length is 1..=255. A long
    /// run-length list may be delivered as several consecutive `Repeat2`
    /// events with the same `val_a`/`val_b`.
    Repeat2 {
        val_a: u32,
        val_b: u32,
        run_lens: &'a [u8],
    },
    /// A tag=0xFA drain tick: firmware wall-clock + backlog telemetry.
    Tick {
        t_us: u32,
        dt_us: u16,
        n_drained: u16,
        n_pending: u16,
        bytes_out: u32,
    },
    /// A tag=0xFD overrun marker: firmware lost `dropped` WR edges.
    Overrun { dropped: u32 },
    /// A tag=0xFE log frame: a UTF-8 line from the firmware.
    Log(&'a str),
    /// `[0xFB]` — firmware acknowledged START.
    Started,
    /// `[0xFC]` — firmware acknowledged STOP.
    Stopped,
}

/// Decode failure. The only recoverable strategy is to resync the stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WireError {
    /// The leading byte of a frame was not a recognised tag.
    BadTag(u8),
}

/// Incremental wire-format decoder with a fixed internal buffer.
pub struct Decoder {
    buf: [u8; BUF_CAP],
    len: usize,
}

impl Default for Decoder {
    fn default() -> Self {
        Self {
            buf: [0; BUF_CAP],
            len: 0,
        }
    }
}

impl Decoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Append `bytes` and drain whatever complete frames are now available,
    /// invoking `on` once per decoded event. Returns `Err(WireError)` on an
    /// unrecognised tag — the caller should resync.
    ///
    /// `bytes` may be any size; bytes that don't yet complete a frame are
    /// retained for the next call. To bound memory, `bytes` is consumed in
    /// pieces that fit the internal buffer, so a single call may process more
    /// than `BUF_CAP` bytes in total.
    pub fn feed(
        &mut self,
        mut bytes: &[u8],
        mut on: impl FnMut(WireEvent<'_>),
    ) -> Result<(), WireError> {
        loop {
            // Top up the internal buffer from the caller's slice.
            let space = BUF_CAP - self.len;
            let take = space.min(bytes.len());
            self.buf[self.len..self.len + take].copy_from_slice(&bytes[..take]);
            self.len += take;
            bytes = &bytes[take..];

            // Drain complete frames out of the buffer.
            self.drain(&mut on)?;

            if bytes.is_empty() {
                return Ok(());
            }
            // More input remains but the buffer didn't free up: a single frame
            // is larger than BUF_CAP and isn't one we can chunk. This only
            // happens on a desynced/garbage stream.
            if self.len == BUF_CAP {
                return Err(WireError::BadTag(self.buf[0]));
            }
        }
    }

    /// Parse and emit as many complete frames as the buffer currently holds,
    /// compacting consumed bytes to the front.
    fn drain(&mut self, on: &mut impl FnMut(WireEvent<'_>)) -> Result<(), WireError> {
        let mut consumed = 0;
        loop {
            match parse_one(&self.buf[consumed..self.len], on)? {
                ParseStep::Consumed(n) => consumed += n,
                ParseStep::NeedMore => break,
            }
        }
        if consumed > 0 {
            self.buf.copy_within(consumed..self.len, 0);
            self.len -= consumed;
        }
        Ok(())
    }
}

enum ParseStep {
    /// A frame was parsed (and emitted); `n` bytes consumed.
    Consumed(usize),
    /// The buffer doesn't hold a full frame yet.
    NeedMore,
}

/// Try to parse one frame from the front of `buf`, emitting it via `on`.
fn parse_one(
    buf: &[u8],
    on: &mut impl FnMut(WireEvent<'_>),
) -> Result<ParseStep, WireError> {
    let Some(&tag) = buf.first() else {
        return Ok(ParseStep::NeedMore);
    };
    match tag {
        TAG_BLOCK => parse_block(buf, on),
        TAG_RUN => {
            if buf.len() < 7 {
                return Ok(ParseStep::NeedMore);
            }
            let n = u16::from_le_bytes([buf[1], buf[2]]);
            let sample = u32::from_le_bytes([buf[3], buf[4], buf[5], buf[6]]);
            on(WireEvent::Run { n, sample });
            Ok(ParseStep::Consumed(7))
        }
        TAG_REPEAT2 => parse_repeat2(buf, on),
        TAG_TICK => {
            if buf.len() < 15 {
                return Ok(ParseStep::NeedMore);
            }
            on(WireEvent::Tick {
                t_us: u32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]),
                dt_us: u16::from_le_bytes([buf[5], buf[6]]),
                n_drained: u16::from_le_bytes([buf[7], buf[8]]),
                n_pending: u16::from_le_bytes([buf[9], buf[10]]),
                bytes_out: u32::from_le_bytes([buf[11], buf[12], buf[13], buf[14]]),
            });
            Ok(ParseStep::Consumed(15))
        }
        TAG_OVERRUN => {
            if buf.len() < 5 {
                return Ok(ParseStep::NeedMore);
            }
            let dropped = u32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]);
            on(WireEvent::Overrun { dropped });
            Ok(ParseStep::Consumed(5))
        }
        TAG_LOG => {
            // UTF-8 text terminated by a NUL byte. Scan for it; wait for more
            // bytes if it isn't here yet.
            let Some(nul_off) = buf[1..].iter().position(|&b| b == 0) else {
                return Ok(ParseStep::NeedMore);
            };
            let text = core::str::from_utf8(&buf[1..1 + nul_off]).unwrap_or("<invalid utf8>");
            on(WireEvent::Log(text));
            Ok(ParseStep::Consumed(1 + nul_off + 1))
        }
        TAG_STARTED => {
            on(WireEvent::Started);
            Ok(ParseStep::Consumed(1))
        }
        TAG_STOPPED => {
            on(WireEvent::Stopped);
            Ok(ParseStep::Consumed(1))
        }
        other => Err(WireError::BadTag(other)),
    }
}

/// Parse a BLOCK frame: `[0x01]` then 4-byte LE samples terminated by the
/// `0xffff_ffff` sentinel. Samples are decoded in place from `buf` and emitted
/// as one `Block` event borrowing that region.
fn parse_block(
    buf: &[u8],
    on: &mut impl FnMut(WireEvent<'_>),
) -> Result<ParseStep, WireError> {
    // Count samples up to the sentinel without consuming partial words.
    let mut off = 1;
    loop {
        if buf.len() < off + 4 {
            return Ok(ParseStep::NeedMore); // need more bytes for the next word
        }
        let w = u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]]);
        off += 4;
        if w == BLOCK_SENTINEL {
            // Samples occupy buf[1..off-4]. Reinterpret in place via a small
            // stack array, emitting in chunks to avoid a large stack frame.
            emit_block_samples(&buf[1..off - 4], on);
            return Ok(ParseStep::Consumed(off));
        }
    }
}

/// Decode the LE-u32 sample bytes in `body` and emit them as `Block` events,
/// chunked through a small stack buffer (no allocation).
fn emit_block_samples(body: &[u8], on: &mut impl FnMut(WireEvent<'_>)) {
    const CHUNK: usize = 64;
    let mut scratch = [0u32; CHUNK];
    let mut i = 0;
    let n = body.len() / 4;
    while i < n {
        let take = CHUNK.min(n - i);
        for (k, slot) in scratch[..take].iter_mut().enumerate() {
            let o = (i + k) * 4;
            *slot = u32::from_le_bytes([body[o], body[o + 1], body[o + 2], body[o + 3]]);
        }
        on(WireEvent::Block(&scratch[..take]));
        i += take;
    }
}

/// Parse a REPEAT2 frame: `[0x03][val_a:u32][val_b:u32][lens..][0x00]`.
fn parse_repeat2(
    buf: &[u8],
    on: &mut impl FnMut(WireEvent<'_>),
) -> Result<ParseStep, WireError> {
    if buf.len() < 9 {
        return Ok(ParseStep::NeedMore);
    }
    let val_a = u32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]);
    let val_b = u32::from_le_bytes([buf[5], buf[6], buf[7], buf[8]]);
    let body_start = 9;
    let Some(zero_off) = buf[body_start..].iter().position(|&b| b == 0) else {
        return Ok(ParseStep::NeedMore);
    };
    let run_lens = &buf[body_start..body_start + zero_off];
    on(WireEvent::Repeat2 {
        val_a,
        val_b,
        run_lens,
    });
    Ok(ParseStep::Consumed(body_start + zero_off + 1))
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate alloc;
    use alloc::vec::Vec;

    /// Owned mirror of `WireEvent` so tests can collect across the borrow.
    #[derive(Debug, Clone, PartialEq, Eq)]
    enum Owned {
        Block(Vec<u32>),
        Run { n: u16, sample: u32 },
        Repeat2 {
            val_a: u32,
            val_b: u32,
            run_lens: Vec<u8>,
        },
        Tick {
            t_us: u32,
            dt_us: u16,
            n_drained: u16,
            n_pending: u16,
            bytes_out: u32,
        },
        Overrun { dropped: u32 },
        Log(alloc::string::String),
        Started,
        Stopped,
    }

    fn own(ev: WireEvent<'_>) -> Owned {
        match ev {
            WireEvent::Block(s) => Owned::Block(s.to_vec()),
            WireEvent::Run { n, sample } => Owned::Run { n, sample },
            WireEvent::Repeat2 {
                val_a,
                val_b,
                run_lens,
            } => Owned::Repeat2 {
                val_a,
                val_b,
                run_lens: run_lens.to_vec(),
            },
            WireEvent::Tick {
                t_us,
                dt_us,
                n_drained,
                n_pending,
                bytes_out,
            } => Owned::Tick {
                t_us,
                dt_us,
                n_drained,
                n_pending,
                bytes_out,
            },
            WireEvent::Overrun { dropped } => Owned::Overrun { dropped },
            WireEvent::Log(t) => Owned::Log(t.into()),
            WireEvent::Started => Owned::Started,
            WireEvent::Stopped => Owned::Stopped,
        }
    }

    /// Feed `bytes` (in one shot) and collect owned events. Coalesce adjacent
    /// `Block` chunks into one so chunking is transparent to assertions.
    fn decode(bytes: &[u8]) -> Vec<Owned> {
        let mut dec = Decoder::new();
        let mut out: Vec<Owned> = Vec::new();
        dec.feed(bytes, |ev| {
            let o = own(ev);
            if let (Some(Owned::Block(prev)), Owned::Block(new)) = (out.last_mut(), &o) {
                prev.extend_from_slice(new);
            } else {
                out.push(o);
            }
        })
        .unwrap();
        out
    }

    fn block_frame(samples: &[u32]) -> Vec<u8> {
        let mut v = alloc::vec![TAG_BLOCK];
        for &s in samples {
            v.extend_from_slice(&s.to_le_bytes());
        }
        v.extend_from_slice(&BLOCK_SENTINEL.to_le_bytes());
        v
    }

    fn run_frame(n: u16, sample: u32) -> Vec<u8> {
        let mut v = alloc::vec![TAG_RUN];
        v.extend_from_slice(&n.to_le_bytes());
        v.extend_from_slice(&sample.to_le_bytes());
        v
    }

    fn repeat2_frame(val_a: u32, val_b: u32, run_lens: &[u8]) -> Vec<u8> {
        let mut v = alloc::vec![TAG_REPEAT2];
        v.extend_from_slice(&val_a.to_le_bytes());
        v.extend_from_slice(&val_b.to_le_bytes());
        v.extend_from_slice(run_lens);
        v.push(0);
        v
    }

    #[test]
    fn empty_yields_nothing() {
        assert!(decode(&[]).is_empty());
    }

    #[test]
    fn single_block() {
        assert_eq!(decode(&block_frame(&[1, 2, 3])), [Owned::Block(alloc::vec![1, 2, 3])]);
    }

    #[test]
    fn run_frame_decodes() {
        assert_eq!(decode(&run_frame(5, 0xAB)), [Owned::Run { n: 5, sample: 0xAB }]);
    }

    #[test]
    fn repeat2_decodes() {
        assert_eq!(
            decode(&repeat2_frame(7, 9, &[2, 3, 2])),
            [Owned::Repeat2 {
                val_a: 7,
                val_b: 9,
                run_lens: alloc::vec![2, 3, 2]
            }]
        );
    }

    #[test]
    fn started_stopped_overrun() {
        assert_eq!(decode(&[TAG_STARTED]), [Owned::Started]);
        assert_eq!(decode(&[TAG_STOPPED]), [Owned::Stopped]);
        assert_eq!(
            decode(&[TAG_OVERRUN, 0x0D, 0x0C, 0x0B, 0x0A]),
            [Owned::Overrun { dropped: 0x0A0B0C0D }]
        );
    }

    #[test]
    fn log_frame_decodes() {
        let mut bytes = alloc::vec![TAG_LOG];
        bytes.extend_from_slice(b"hi");
        bytes.push(0);
        assert_eq!(decode(&bytes), [Owned::Log("hi".into())]);
    }

    #[test]
    fn tick_decodes() {
        let mut bytes = alloc::vec![TAG_TICK];
        bytes.extend_from_slice(&0x11223344u32.to_le_bytes());
        bytes.extend_from_slice(&0x5566u16.to_le_bytes());
        bytes.extend_from_slice(&0x7788u16.to_le_bytes());
        bytes.extend_from_slice(&0x99AAu16.to_le_bytes());
        bytes.extend_from_slice(&0xBBCCDDEEu32.to_le_bytes());
        assert_eq!(
            decode(&bytes),
            [Owned::Tick {
                t_us: 0x11223344,
                dt_us: 0x5566,
                n_drained: 0x7788,
                n_pending: 0x99AA,
                bytes_out: 0xBBCCDDEE,
            }]
        );
    }

    #[test]
    fn multiple_frames_in_one_feed() {
        let mut bytes = block_frame(&[1, 2]);
        bytes.extend_from_slice(&run_frame(3, 9));
        bytes.extend_from_slice(&block_frame(&[4]));
        assert_eq!(
            decode(&bytes),
            [
                Owned::Block(alloc::vec![1, 2]),
                Owned::Run { n: 3, sample: 9 },
                Owned::Block(alloc::vec![4]),
            ]
        );
    }

    #[test]
    fn partial_frame_split_across_feeds() {
        // Split a RUN frame across two feed calls byte-by-byte; it must only
        // emit once fully delivered.
        let frame = run_frame(42, 0xDEADBEEF);
        let mut dec = Decoder::new();
        let mut out: Vec<Owned> = Vec::new();
        for chunk in frame.chunks(1) {
            dec.feed(chunk, |ev| out.push(own(ev))).unwrap();
        }
        assert_eq!(out, [Owned::Run { n: 42, sample: 0xDEADBEEF }]);
    }

    #[test]
    fn block_split_mid_word() {
        // Feed a block frame split in the middle of a sample word.
        let frame = block_frame(&[0x01020304, 0x05060708]);
        let mut dec = Decoder::new();
        let mut out: Vec<Owned> = Vec::new();
        let (a, b) = frame.split_at(4); // mid first sample
        dec.feed(a, |ev| {
            let o = own(ev);
            coalesce(&mut out, o);
        })
        .unwrap();
        dec.feed(b, |ev| {
            let o = own(ev);
            coalesce(&mut out, o);
        })
        .unwrap();
        assert_eq!(out, [Owned::Block(alloc::vec![0x01020304, 0x05060708])]);
    }

    fn coalesce(out: &mut Vec<Owned>, o: Owned) {
        if let (Some(Owned::Block(prev)), Owned::Block(new)) = (out.last_mut(), &o) {
            prev.extend_from_slice(new);
        } else {
            out.push(o);
        }
    }

    #[test]
    fn bad_tag_errors() {
        let mut dec = Decoder::new();
        let err = dec.feed(&[0x55], |_| {}).unwrap_err();
        assert_eq!(err, WireError::BadTag(0x55));
    }

    #[test]
    fn large_block_chunks_then_coalesces() {
        // A block larger than the CHUNK stack buffer must round-trip when
        // chunks are coalesced.
        let samples: Vec<u32> = (0..200).collect();
        let got = decode(&block_frame(&samples));
        assert_eq!(got, [Owned::Block(samples)]);
    }
}
