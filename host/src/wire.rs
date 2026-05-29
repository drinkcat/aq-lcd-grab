//! Wire-format parser for the capture board → host stream.
//!
//! Mirrors the encoder in `firmware/src/wire.rs` and
//! `firmware-stm32/src/wire.rs`; see `docs/wire_protocol.md` for the
//! framing spec. Feed bytes in via [`Decoder::feed`]; pull decoded
//! [`Event`]s back. The decoder keeps any partial trailing frame for
//! the next call, so the caller can pass whatever-size reads the OS
//! returns.
//!
//! Frame integrity is the firmware's job (it stages frames atomically
//! into its TX pipe). If the host ever sees an unrecognised tag, that
//! means the stream desynced — `feed` returns `Err`, the caller is
//! expected to resync with STOP/drain/START.

use std::io;

pub const TAG_BLOCK: u8 = 0x01;
pub const TAG_RUN: u8 = 0x02;
pub const TAG_REPEAT2: u8 = 0x03;
pub const TAG_TICK: u8 = 0xFA;
pub const TAG_OVERRUN: u8 = 0xFD;
pub const TAG_LOG: u8 = 0xFE;
pub const TAG_STARTED: u8 = 0xFB;
pub const TAG_STOPPED: u8 = 0xFC;

pub const HOST_CMD_START: u8 = 0x01;
pub const HOST_CMD_STOP: u8 = 0x02;

/// One decoded event from the wire.
///
/// Sample layout: low 16 bits = `pa` (GPIOA->IDR), high 16 bits = `pb`
/// (GPIOB->IDR). Matches the firmware's packed-u32 encoder exactly —
/// the on-wire bytes are `to_le_bytes()` of this u32. Per-board
/// permute layer splits/unscrambles into (data, dc, cs).
#[derive(Debug, Clone)]
pub enum Event {
    /// A tag=0x01 block, expanded into raw samples.
    Block(Vec<u32>),
    /// A tag=0x02 run: `n` repetitions of `sample`.
    Run { n: u16, sample: u32 },
    /// A tag=0x03 REPEAT2: a sequence of runs alternating between
    /// `val_a` and `val_b`. `run_lens[i]` is the length of run `i`;
    /// run 0 is `val_a`, run 1 is `val_b`, run 2 is `val_a`, etc.
    /// Each length is 1..=255 (a lone sample may participate).
    Repeat2 {
        val_a: u32,
        val_b: u32,
        run_lens: Vec<u8>,
    },
    /// A tag=0xFA drain tick: firmware wall-clock + backlog telemetry.
    Tick {
        /// Firmware `Instant::now()` at the start of the drain pass
        /// (low 32 bits, µs). Wraps every ~71 minutes.
        t_us: u32,
        /// Wall-clock duration of the drain pass (`t1 - t0`, µs).
        dt_us: u16,
        /// Samples consumed in this drain pass.
        n_drained: u16,
        /// Samples still pending in the PIO/DMA ring after drain.
        n_pending: u16,
        /// Bytes the firmware enqueued to the USB CDC stream during
        /// this TICK window, *excluding* TICK frames themselves.
        /// Per-window delta (not cumulative). Compare against
        /// `n_drained * 4` for the encoder's effective compression
        /// ratio over the window.
        bytes_out: u32,
    },
    /// A tag=0xFD overrun marker: firmware lost `dropped` WR edges.
    Overrun { dropped: u32 },
    /// A tag=0xFE log frame: a UTF-8 line from the firmware.
    Log(String),
    /// `[0xFB]` — firmware acknowledged START.
    Started,
    /// `[0xFC]` — firmware acknowledged STOP.
    Stopped,
}

/// Incremental wire-format decoder.
#[derive(Default)]
pub struct Decoder {
    buf: Vec<u8>,
}

impl Decoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Append `bytes` and drain whatever complete frames are now available.
    /// Returns `Err` on an unrecognised tag — the caller should resync.
    pub fn feed(&mut self, bytes: &[u8]) -> io::Result<Vec<Event>> {
        self.buf.extend_from_slice(bytes);
        let mut events = Vec::new();
        let mut consumed = 0usize;
        loop {
            let rest = &self.buf[consumed..];
            match parse_one(rest)? {
                Some((ev, n)) => {
                    events.push(ev);
                    consumed += n;
                }
                None => break,
            }
        }
        if consumed > 0 {
            self.buf.drain(..consumed);
        }
        Ok(events)
    }
}

/// Try to parse one frame from `buf`. Returns `Ok(Some((ev, n)))` with
/// the event and bytes consumed, `Ok(None)` if the buffer doesn't yet
/// hold a full frame, or `Err` if the leading byte isn't a valid tag.
fn parse_one(buf: &[u8]) -> io::Result<Option<(Event, usize)>> {
    let Some(&tag) = buf.first() else {
        return Ok(None);
    };
    match tag {
        TAG_BLOCK => {
            // Sample list of 4-byte LE u32s, terminated by the
            // 0xffff_ffff sentinel. No leading count.
            let mut samples = Vec::new();
            let mut off = 1;
            loop {
                if buf.len() < off + 4 {
                    return Ok(None); // need more bytes for the next word
                }
                let w = u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]]);
                off += 4;
                if w == 0xffff_ffff {
                    return Ok(Some((Event::Block(samples), off)));
                }
                samples.push(w);
            }
        }
        TAG_RUN => {
            if buf.len() < 7 {
                return Ok(None);
            }
            let n = u16::from_le_bytes([buf[1], buf[2]]);
            let sample = u32::from_le_bytes([buf[3], buf[4], buf[5], buf[6]]);
            Ok(Some((Event::Run { n, sample }, 7)))
        }
        TAG_TICK => {
            if buf.len() < 15 {
                return Ok(None);
            }
            let t_us = u32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]);
            let dt_us = u16::from_le_bytes([buf[5], buf[6]]);
            let n_drained = u16::from_le_bytes([buf[7], buf[8]]);
            let n_pending = u16::from_le_bytes([buf[9], buf[10]]);
            let bytes_out = u32::from_le_bytes([buf[11], buf[12], buf[13], buf[14]]);
            Ok(Some((
                Event::Tick {
                    t_us,
                    dt_us,
                    n_drained,
                    n_pending,
                    bytes_out,
                },
                15,
            )))
        }
        TAG_REPEAT2 => {
            // Header: tag + val_a:u32 + val_b:u32 = 9 bytes.
            if buf.len() < 9 {
                return Ok(None);
            }
            let val_a = u32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]);
            let val_b = u32::from_le_bytes([buf[5], buf[6], buf[7], buf[8]]);
            // Scan the rest for the null terminator. If not yet
            // present, the frame's incomplete — wait for more bytes.
            let body_start = 9;
            let Some(zero_off) = buf[body_start..].iter().position(|&b| b == 0) else {
                return Ok(None);
            };
            let run_lens = buf[body_start..body_start + zero_off].to_vec();
            let consumed = body_start + zero_off + 1;
            Ok(Some((
                Event::Repeat2 {
                    val_a,
                    val_b,
                    run_lens,
                },
                consumed,
            )))
        }
        TAG_OVERRUN => {
            if buf.len() < 5 {
                return Ok(None);
            }
            let dropped = u32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]);
            Ok(Some((Event::Overrun { dropped }, 5)))
        }
        TAG_LOG => {
            // UTF-8 text terminated by a NUL byte (never present in the
            // firmware's log text). Scan for it; wait for more bytes if
            // it isn't here yet.
            let Some(nul_off) = buf[1..].iter().position(|&b| b == 0) else {
                return Ok(None);
            };
            let msg = String::from_utf8_lossy(&buf[1..1 + nul_off]).into_owned();
            Ok(Some((Event::Log(msg), 1 + nul_off + 1)))
        }
        TAG_STARTED => Ok(Some((Event::Started, 1))),
        TAG_STOPPED => Ok(Some((Event::Stopped, 1))),
        other => Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("unrecognised wire tag {other:#04x}"),
        )),
    }
}
