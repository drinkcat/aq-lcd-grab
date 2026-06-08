//! The on-chip decode pipeline: wire bytes → samples → permute →
//! glyph decoder + framebuffer.
//!
//! Mirrors the host's `dispatch_event` flow but allocation-free. One
//! [`Pipeline`] owns the wire decoder, glyph decoder, and a handle to the
//! shared framebuffer; [`Pipeline::feed`] consumes a byte buffer from the UART.

use embassy_sync::pubsub::Publisher;
use log::{info, warn};
use wire::{WireError, WireEvent};

use crate::{RowUpdate, SharedFb};

/// Permutation from a raw packed sample to `(data, is_data)`. Selected by build
/// feature: STM32 bench rig (default) or Pico.
#[cfg(feature = "bridge-pico")]
const PERMUTE: fn(u32) -> (u16, bool) = wire::permute_pico;
#[cfg(not(feature = "bridge-pico"))]
const PERMUTE: fn(u32) -> (u16, bool) = wire::permute_f103;

type ValuesPub = Publisher<
    'static,
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    RowUpdate,
    8,
    1,
    1,
>;

pub struct Pipeline {
    wire: wire::Decoder,
    glyph: decoder::Decoder,
}

impl Pipeline {
    pub fn new() -> Self {
        Self {
            wire: wire::Decoder::new(),
            glyph: decoder::Decoder::new(),
        }
    }

    /// Feed a buffer of wire bytes. Drives the glyph decoder and framebuffer for
    /// every decoded sample, and flushes settled rows to `values_pub` on each
    /// firmware LOG frame (the host uses the same flush points). Returns `Err`
    /// if the wire stream desyncs — the caller should resync (STOP/START).
    pub async fn feed(
        &mut self,
        bytes: &[u8],
        fb: &SharedFb,
        values_pub: &ValuesPub,
    ) -> Result<(), WireError> {
        // The wire decoder's callback can't be async, so we collect intent into
        // small flags and act after each `feed` returns. To keep the framebuffer
        // lock short and avoid awaiting inside the callback, we lock the fb for
        // the whole buffer (UART buffers are small, ~256 B).
        let mut fb = fb.lock().await;
        let glyph = &mut self.glyph;
        let mut flush_requested = false;

        let res = self.wire.feed(bytes, |ev| match ev {
            WireEvent::Block(samples) => {
                for &s in samples {
                    let (data, is_data) = PERMUTE(s);
                    glyph.feed(data, is_data);
                    fb.feed(data, is_data);
                }
            }
            WireEvent::Run { n, sample } => {
                let (data, is_data) = PERMUTE(sample);
                for _ in 0..n {
                    glyph.feed(data, is_data);
                    fb.feed(data, is_data);
                }
            }
            WireEvent::Repeat2 {
                val_a,
                val_b,
                run_lens,
            } => {
                let a = PERMUTE(val_a);
                let b = PERMUTE(val_b);
                for (i, &len) in run_lens.iter().enumerate() {
                    let (data, is_data) = if i & 1 == 0 { a } else { b };
                    for _ in 0..len {
                        glyph.feed(data, is_data);
                        fb.feed(data, is_data);
                    }
                }
            }
            WireEvent::Log(_) => {
                // Firmware log lines fall between display refreshes — good flush
                // points (same as the host). Defer the actual flush until after
                // the borrow of `glyph`/`fb` ends.
                flush_requested = true;
            }
            WireEvent::Overrun { dropped } => warn!("bridge lost {dropped} samples"),
            WireEvent::Tick { .. } | WireEvent::Started | WireEvent::Stopped => {}
        });

        if res.is_err() {
            // Drop the buffered garbage; the caller re-syncs the bridge.
            self.wire.reset();
        }

        if flush_requested {
            glyph.flush_each(|name, value| {
                let mut v = heapless::String::<16>::new();
                let _ = v.push_str(value);
                info!("= {name}: {value}");
                // Non-blocking publish; drop if the queue is full (MQTT is slow).
                values_pub.publish_immediate(RowUpdate { name, value: v });
            });
        }

        res
    }
}

impl Default for Pipeline {
    fn default() -> Self {
        Self::new()
    }
}

/// START/STOP handshake bytes for the bridge. Re-exported for the UART task's
/// sync sequence.
pub use wire::{HOST_CMD_START, HOST_CMD_STOP};
