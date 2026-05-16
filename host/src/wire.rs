//! Wire-format parser (mirror of `firmware/src/proto.rs`).
//!
//! Reads framed transactions from a byte stream:
//!
//!     [0xAA] [0x55]            magic
//!     [cmd  u8 ]                command byte (0xFF = log line)
//!     [count u16 LE]            data-word count, high bit = RLE flag
//!     [data  u16 LE × N]        payload (raw, or (len, value) pairs)
//!
//! On stream resync, we scan for the magic prefix and resume.

use std::io::{self, Read};

pub const MAGIC_0: u8 = 0xAA;
pub const MAGIC_1: u8 = 0x55;
pub const CMD_LOG: u8 = 0xFF;
pub const RLE_FLAG: u16 = 0x8000;
pub const COUNT_MASK: u16 = 0x7FFF;

#[derive(Clone, Debug)]
pub struct Frame {
    pub cmd: u8,
    pub data: Vec<u16>,
}

/// Read one frame from `r`. Re-syncs by scanning for the magic prefix.
/// RLE-encoded frames are expanded into raw pixel words before return,
/// so downstream code sees a uniform `data: Vec<u16>` payload either way.
pub fn read_frame<R: Read>(r: &mut R) -> io::Result<Frame> {
    // Find the magic.
    let mut prev = 0u8;
    let mut byte = [0u8; 1];
    loop {
        r.read_exact(&mut byte)?;
        if prev == MAGIC_0 && byte[0] == MAGIC_1 {
            break;
        }
        prev = byte[0];
    }

    let mut cmd_buf = [0u8; 1];
    r.read_exact(&mut cmd_buf)?;
    let cmd = cmd_buf[0];

    let mut count_buf = [0u8; 2];
    r.read_exact(&mut count_buf)?;
    let count_raw = u16::from_le_bytes(count_buf);
    let is_rle = count_raw & RLE_FLAG != 0;
    let count = (count_raw & COUNT_MASK) as usize;

    let mut raw = Vec::with_capacity(count);
    let mut word_buf = [0u8; 2];
    for _ in 0..count {
        r.read_exact(&mut word_buf)?;
        raw.push(u16::from_le_bytes(word_buf));
    }

    let data = if is_rle {
        let mut out = Vec::new();
        for pair in raw.chunks_exact(2) {
            let len = pair[0] as usize;
            let value = pair[1];
            out.resize(out.len() + len, value);
        }
        out
    } else {
        raw
    };

    Ok(Frame { cmd, data })
}

/// Decode a CMD_LOG frame's data as a UTF-8 string.
pub fn log_text(data: &[u16]) -> String {
    let mut bytes = Vec::with_capacity(data.len() * 2);
    for &w in data {
        let b = w.to_le_bytes();
        bytes.push(b[0]);
        bytes.push(b[1]);
    }
    // Strip trailing zeros (padding from odd-length original messages).
    while bytes.last() == Some(&0) {
        bytes.pop();
    }
    String::from_utf8_lossy(&bytes).into_owned()
}
