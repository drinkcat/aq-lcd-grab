#![no_std]
#![feature(impl_trait_in_assoc_type)]
#![recursion_limit = "256"]

//! Shared types for the ESP32-C6 gateway: the decode pipeline and the
//! cross-task channels/state.

pub mod http;
pub mod logger;
#[cfg(feature = "homeassistant")]
pub mod mqtt;
pub mod pipeline;
pub mod reboot;

extern crate alloc;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::PubSubChannel;
use framebuffer::{Framebuffer, Palette4Store};

/// A decoded metric update: row name (static) + value string (e.g. "3.3").
#[derive(Clone)]
pub struct RowUpdate {
    pub name: &'static str,
    pub value: heapless::String<16>,
}

/// Values pub/sub: the UART task publishes [`RowUpdate`]s; the MQTT task
/// subscribes. Capacity 8, 1 subscriber (mqtt), 1 publisher (uart).
pub static VALUES: PubSubChannel<CriticalSectionRawMutex, RowUpdate, 8, 1, 1> =
    PubSubChannel::new();

/// The metric rows, in a fixed order. Mirrors `decoder::ROWS`; used as the
/// schema for the latest-values snapshot and the `/values` JSON.
pub const ROW_NAMES: [&str; 5] = ["pm25", "tvoc", "co2", "temp", "humidity"];

/// Latest decoded value per row, indexed parallel to [`ROW_NAMES`]. The UART
/// pipeline updates it; the HTTP `/values` handler reads it. Empty = not seen
/// yet. Separate from [`VALUES`] (which is consume-once) so HTTP can sample the
/// current state on demand.
pub type LatestValues = Mutex<CriticalSectionRawMutex, [heapless::String<16>; 5]>;

/// Record a decoded value into `latest` by row name (no-op if the name is
/// unknown).
pub async fn record_value(latest: &LatestValues, name: &str, value: &str) {
    if let Some(i) = ROW_NAMES.iter().position(|&n| n == name) {
        let mut g = latest.lock().await;
        g[i].clear();
        let _ = g[i].push_str(value);
    }
}

/// The reconstructed panel framebuffer, shared between the UART task (writer)
/// and the HTTP task (reader). Backed by a `'static` palette buffer.
pub type SharedFb = Mutex<CriticalSectionRawMutex, Framebuffer<Palette4Store<'static>>>;
