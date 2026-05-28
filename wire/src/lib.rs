//! Shared wire-protocol encoder/decoder for the display-bus capture
//! boards (Pico, STM32) and the host viewer.
//!
//! The crate is `no_std` so both firmware targets can link it; tests
//! run on the host with `std` available via the test harness.
//!
//! A capture board turns a stream of packed `(pa, pb)` samples into a
//! byte sequence of tagged frames. The host parses them back. Keeping
//! both sides in one crate means the encoder and decoder can't drift,
//! and round-trip tests run natively.

#![cfg_attr(not(test), no_std)]

mod encoder;
mod sink;

pub use encoder::{
    Encoder, TAG_BLOCK, TAG_LOG, TAG_OVERRUN, TAG_RUN, TAG_STARTED, TAG_STOPPED, TAG_TICK,
};
pub use sink::Sink;

// Host → board control commands (single bytes on the reverse channel).
/// Begin streaming.
pub const HOST_CMD_START: u8 = 0x01;
/// Halt streaming and drain.
pub const HOST_CMD_STOP: u8 = 0x02;
/// Round-trip health check: board replies with a `"ping"` log frame.
pub const HOST_CMD_LOG_TEST: u8 = 0x03;
/// Dump capture counters as log frame(s).
pub const HOST_CMD_STATS: u8 = 0x04;
