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
pub const TAG_TICK: u8 = 0x03;
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
    /// A tag=0x03 drain tick: firmware wall-clock + backlog telemetry.
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
            if buf.len() < 2 {
                return Ok(None);
            }
            let n = buf[1] as usize;
            let needed = 2 + 4 * n;
            if buf.len() < needed {
                return Ok(None);
            }
            let mut samples = Vec::with_capacity(n);
            for i in 0..n {
                let off = 2 + 4 * i;
                let s = u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]]);
                samples.push(s);
            }
            Ok(Some((Event::Block(samples), needed)))
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
            if buf.len() < 11 {
                return Ok(None);
            }
            let t_us = u32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]);
            let dt_us = u16::from_le_bytes([buf[5], buf[6]]);
            let n_drained = u16::from_le_bytes([buf[7], buf[8]]);
            let n_pending = u16::from_le_bytes([buf[9], buf[10]]);
            Ok(Some((Event::Tick { t_us, dt_us, n_drained, n_pending }, 11)))
        }
        TAG_OVERRUN => {
            if buf.len() < 5 {
                return Ok(None);
            }
            let dropped = u32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]);
            Ok(Some((Event::Overrun { dropped }, 5)))
        }
        TAG_LOG => {
            if buf.len() < 3 {
                return Ok(None);
            }
            let len = u16::from_le_bytes([buf[1], buf[2]]) as usize;
            let needed = 3 + len;
            if buf.len() < needed {
                return Ok(None);
            }
            let msg = String::from_utf8_lossy(&buf[3..needed]).into_owned();
            Ok(Some((Event::Log(msg), needed)))
        }
        TAG_STARTED => Ok(Some((Event::Started, 1))),
        TAG_STOPPED => Ok(Some((Event::Stopped, 1))),
        other => Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("unrecognised wire tag {other:#04x}"),
        )),
    }
}
