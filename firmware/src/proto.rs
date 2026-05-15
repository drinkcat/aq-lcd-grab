//! Wire protocol between firmware and host.
//!
//! Each frame on the USB CDC stream:
//!
//!     [0xAA] [0x55]            magic (resync)
//!     [cmd  u8 ]                ILI9488 command byte, or 0xFF for log lines
//!     [count u16 LE]            number of data words that follow
//!     [data  u16 LE × count]    payload
//!
//! Length of one frame: 5 + 2 * count bytes.
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

/// Maximum data words per single frame. Keeps individual writes bounded
/// and avoids long synchronous USB sends in the capture loop.
pub const MAX_DATA_WORDS: u16 = 4096;

/// Encode a frame header into 5 bytes.
pub fn encode_header(cmd: u8, count: u16) -> [u8; 5] {
    let bytes = count.to_le_bytes();
    [MAGIC_0, MAGIC_1, cmd, bytes[0], bytes[1]]
}
