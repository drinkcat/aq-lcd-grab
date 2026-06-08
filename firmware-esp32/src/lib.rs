#![no_std]

//! Shared types for the ESP32-C6 gateway: the decode pipeline and the
//! cross-task channels/state.

pub mod pipeline;

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

/// The reconstructed panel framebuffer, shared between the UART task (writer)
/// and the HTTP task (reader). Backed by a `'static` palette buffer.
pub type SharedFb = Mutex<CriticalSectionRawMutex, Framebuffer<Palette4Store<'static>>>;
