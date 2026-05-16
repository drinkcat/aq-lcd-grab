//! Wire protocol between firmware and host.
//!
//! Each frame on the USB CDC stream:
//!
//!     [0xAA] [0x55]            magic (resync)
//!     [cmd  u8 ]                ILI9488 command byte, or 0xFF for log lines
//!     [count u16 LE]            data-word count + flags (see below)
//!     [data  u16 LE × N]        payload
//!
//! The low 15 bits of `count` give the number of u16 words that follow.
//! The high bit (RLE_FLAG = 0x8000) means the data is run-length-encoded
//! as `(length, value)` u16 pairs; the total pixel count is the sum of
//! the lengths. Without the flag, data is raw pixel words as captured.
//!
//! For a MEMORY_WRITE longer than 65535 pixels, the firmware splits it
//! into multiple sub-transactions: one with cmd=0x2C (or 0x3C), then
//! additional frames with cmd=0x3C (MEMORY_WRITE_CONTINUE).
//!
//! Log lines use cmd=0xFF; the data bytes are interpreted as raw UTF-8
//! (two chars per u16, little-endian). Host strips trailing NULs.

pub const MAGIC_0: u8 = 0xAA;
pub const MAGIC_1: u8 = 0x55;

pub const CMD_LOG: u8 = 0xFF;
pub const CMD_MEMORY_WRITE_CONTINUE: u8 = 0x3C;

/// High bit of the `count` field: data is RLE-encoded `(len, value)` pairs.
pub const RLE_FLAG: u16 = 0x8000;

/// Maximum data words per single frame. Keeps individual writes bounded
/// and avoids long synchronous USB sends in the capture loop.
pub const MAX_DATA_WORDS: u16 = 4096;

/// Encode a frame header into 5 bytes. `count_with_flags` already has
/// any flag bits (e.g. RLE_FLAG) OR'd into the word count.
pub fn encode_header(cmd: u8, count_with_flags: u16) -> [u8; 5] {
    let bytes = count_with_flags.to_le_bytes();
    [MAGIC_0, MAGIC_1, cmd, bytes[0], bytes[1]]
}
