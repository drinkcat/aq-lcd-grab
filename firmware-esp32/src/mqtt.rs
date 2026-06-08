//! MQTT publisher: push decoded sensor values to Home Assistant.
//!
//! Subscribes to the [`crate::VALUES`] pubsub (fed by the decode pipeline) and
//! publishes each row to its state topic under `aq/`. On (re)connect it also
//! publishes MQTT discovery configs so Home Assistant auto-creates the entities.
//!
//! embassy-net 0.9's `TcpSocket` implements `embedded-io-async` 0.7 directly —
//! the version rust-mqtt wants — so no version-bridging shim is needed.

use embassy_net::Stack;
use embassy_net::tcp::TcpSocket;
use embassy_time::{Duration, Timer};
use log::info;
use static_cell::StaticCell;

use crate::VALUES;

const HA_HOST: &str = env!("HA_HOST");
const HA_USER: &str = env!("HA_USER");
const HA_TOKEN: &str = env!("HA_TOKEN");

/// One sensor's Home Assistant wiring: the row name (matches the decoder and
/// state topic), its discovery config topic + payload, and its state topic.
struct Sensor {
    row: &'static str,
    disc_topic: &'static str,
    disc_payload: &'static str,
    state_topic: &'static str,
}

// The panel reports PM2.5 (µg/m³), TVOC (ppm per the panel label), CO2 (ppm),
// temperature (°C) and humidity (%). State topics live under `aq/` (subscribe
// to `aq/#`); discovery stays under `homeassistant/` as HA requires. All share
// one device so HA groups them.
const SENSORS: &[Sensor] = &[
    Sensor {
        row: "pm25",
        disc_topic: "homeassistant/sensor/aq/pm25/config",
        disc_payload: r#"{"name":"PM2.5","uniq_id":"aq_pm25","stat_t":"aq/pm25","dev_cla":"pm25","unit_of_meas":"µg/m³","stat_cla":"measurement","dev":{"ids":["aq"],"name":"Air Quality"}}"#,
        state_topic: "aq/pm25",
    },
    Sensor {
        row: "tvoc",
        disc_topic: "homeassistant/sensor/aq/tvoc/config",
        disc_payload: r#"{"name":"TVOC","uniq_id":"aq_tvoc","stat_t":"aq/tvoc","dev_cla":"volatile_organic_compounds_parts","unit_of_meas":"ppm","stat_cla":"measurement","dev":{"ids":["aq"],"name":"Air Quality"}}"#,
        state_topic: "aq/tvoc",
    },
    Sensor {
        row: "co2",
        disc_topic: "homeassistant/sensor/aq/co2/config",
        disc_payload: r#"{"name":"CO2","uniq_id":"aq_co2","stat_t":"aq/co2","dev_cla":"carbon_dioxide","unit_of_meas":"ppm","stat_cla":"measurement","dev":{"ids":["aq"],"name":"Air Quality"}}"#,
        state_topic: "aq/co2",
    },
    Sensor {
        row: "temp",
        disc_topic: "homeassistant/sensor/aq/temp/config",
        disc_payload: r#"{"name":"Temperature","uniq_id":"aq_temp","stat_t":"aq/temp","dev_cla":"temperature","unit_of_meas":"°C","stat_cla":"measurement","dev":{"ids":["aq"],"name":"Air Quality"}}"#,
        state_topic: "aq/temp",
    },
    Sensor {
        row: "humidity",
        disc_topic: "homeassistant/sensor/aq/humidity/config",
        disc_payload: r#"{"name":"Humidity","uniq_id":"aq_humidity","stat_t":"aq/humidity","dev_cla":"humidity","unit_of_meas":"%","stat_cla":"measurement","dev":{"ids":["aq"],"name":"Air Quality"}}"#,
        state_topic: "aq/humidity",
    },
];

/// MQTT publisher task: connect to the HA broker, publish discovery + values,
/// reconnecting on any error.
#[embassy_executor::task]
pub async fn mqtt_task(stack: Stack<'static>) {
    use core::num::NonZero;
    use rust_mqtt::{
        Bytes,
        buffer::AllocBuffer,
        client::{
            Client,
            options::{ConnectOptions, PublicationOptions, TopicReference},
        },
        config::KeepAlive,
        types::{MqttBinary, MqttString, TopicName},
    };

    let mut sub = VALUES.subscriber().unwrap();

    // Socket buffers are allocated once and reused across reconnects — a
    // StaticCell can only be init'd once, so they must live outside the loop.
    let rx_buf = {
        static RX: StaticCell<[u8; 1024]> = StaticCell::new();
        RX.init([0; 1024])
    };
    let tx_buf = {
        static TX: StaticCell<[u8; 1024]> = StaticCell::new();
        TX.init([0; 1024])
    };

    loop {
        info!("MQTT connecting to {HA_HOST}...");
        let mut sock = TcpSocket::new(stack, &mut rx_buf[..], &mut tx_buf[..]);
        sock.set_timeout(Some(Duration::from_secs(10)));

        let remote = match stack
            .dns_query(HA_HOST, embassy_net::dns::DnsQueryType::A)
            .await
        {
            Ok(addrs) if !addrs.is_empty() => embassy_net::IpEndpoint::new(addrs[0], 1883),
            _ => {
                info!("MQTT DNS failed, retrying in 10s");
                Timer::after(Duration::from_secs(10)).await;
                continue;
            }
        };

        if let Err(e) = sock.connect(remote).await {
            info!("MQTT TCP connect failed: {e:?}, retrying in 10s");
            Timer::after(Duration::from_secs(10)).await;
            continue;
        }

        let mut buffer = AllocBuffer;
        let mut client = Client::<'_, _, _, 0, 1, 1, 0>::new(&mut buffer);

        let connect_opts = ConnectOptions::new()
            .clean_start()
            .keep_alive(KeepAlive::Seconds(NonZero::new(60).unwrap()))
            .user_name(MqttString::try_from(HA_USER).unwrap())
            .password(MqttBinary::try_from(HA_TOKEN.as_bytes()).unwrap());

        match client
            .connect(
                sock,
                &connect_opts,
                Some(MqttString::try_from("aq_lcd").unwrap()),
            )
            .await
        {
            Ok(_) => info!("MQTT connected"),
            Err(e) => {
                info!("MQTT connect failed: {e:?}, retrying in 10s");
                Timer::after(Duration::from_secs(10)).await;
                continue;
            }
        }

        // Publish discovery configs (retained) so HA auto-creates the entities.
        let mut disc_ok = true;
        for s in SENSORS {
            let topic = TopicName::new(MqttString::try_from(s.disc_topic).unwrap()).unwrap();
            let opts = PublicationOptions::new(TopicReference::Name(topic)).retain();
            if let Err(e) = client
                .publish(&opts, Bytes::from(s.disc_payload.as_bytes()))
                .await
            {
                info!("MQTT discovery publish failed ({}): {e:?}", s.row);
                disc_ok = false;
                break;
            }
        }
        if !disc_ok {
            Timer::after(Duration::from_secs(5)).await;
            continue;
        }
        info!("MQTT discovery published");

        // The pipeline re-flushes a row every time the panel repaints the same
        // digits (many times/second), so publish only when a value actually
        // changes — otherwise we'd spam the broker and HA's recorder with
        // identical retained messages. Retained means HA keeps the last value
        // across our silence, so no heartbeat is needed. `last` is cleared each
        // (re)connect so the first sample after connecting always republishes.
        let mut last: [heapless::String<16>; SENSORS.len()] = Default::default();

        loop {
            let update = sub.next_message_pure().await;
            let Some(idx) = SENSORS.iter().position(|s| s.row == update.name) else {
                continue;
            };
            if last[idx] == update.value {
                continue; // unchanged — skip the publish
            }

            let topic =
                TopicName::new(MqttString::try_from(SENSORS[idx].state_topic).unwrap()).unwrap();
            let opts = PublicationOptions::new(TopicReference::Name(topic)).retain();
            match client
                .publish(&opts, Bytes::from(update.value.as_bytes()))
                .await
            {
                Ok(_) => {
                    info!("MQTT {} = {}", update.name, update.value.as_str());
                    last[idx].clear();
                    let _ = last[idx].push_str(&update.value);
                }
                Err(e) => {
                    info!("MQTT publish failed: {e:?}, reconnecting");
                    break;
                }
            }
        }
    }
}
