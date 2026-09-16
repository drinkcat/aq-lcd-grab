//! MQTT publisher: push decoded sensor values to Home Assistant.
//!
//! Subscribes to the [`crate::VALUES`] pubsub (fed by the decode pipeline) and
//! publishes each row to its state topic under `aq/`. On (re)connect it also
//! publishes MQTT discovery configs so Home Assistant auto-creates the entities.
//!
//! embassy-net 0.9's `TcpSocket` implements `embedded-io-async` 0.7 directly —
//! the version rust-mqtt wants — so no version-bridging shim is needed.

use embassy_futures::select::{Either4, select4};
use embassy_net::Stack;
use embassy_net::tcp::TcpSocket;
use embassy_time::{Duration, Instant, Timer};
use log::info;
use static_cell::StaticCell;

use crate::VALUES;

unsafe extern "C" {
    fn esp_rom_software_reset_system();
}

/// Check if a value string looks malformed (leading 0 without dot, `#` chars, gaps).
fn is_value_malformed(s: &str) -> bool {
    if s.is_empty() {
        return false; // empty is OK (not seen yet)
    }
    if s.contains('#') {
        return true; // unrecognized glyph
    }
    if s.contains("  ") {
        return true; // gap (multiple spaces)
    }
    // Leading 0 without decimal point (e.g. "012" but not "0.1")
    if s.starts_with('0') && !s.contains('.') && s.len() > 1 {
        return true;
    }
    false
}

const HA_HOST: &str = env!("HA_HOST");
const HA_USER: &str = env!("HA_USER");
const HA_TOKEN: &str = env!("HA_TOKEN");

/// MQTT client id, derived from this unit's `AQ_ID`. It must differ per unit: a
/// broker drops the existing session when a second client connects with the
/// same id, so two units sharing one id would disconnect each other in a loop.
const CLIENT_ID: &str = concat!("aq_lcd_", env!("AQ_ID"));

// MQTT keepalive advertised to the broker, and how often we ping. The ping
// must fire well within the keepalive, and the TCP socket timeout must exceed
// the ping interval, or the connection tears down between publishes.
const MQTT_KEEPALIVE_SECS: u16 = 60;
const PING_INTERVAL: Duration = Duration::from_secs(30);

// The panel updates its digits one at a time, so during a change like 605 -> 599
// the display transiently reads an intermediate value (e.g. 505, when only the
// hundreds digit has flipped). Debounce: hold a changed value back until it has
// stayed stable for this long before publishing, so those transients are dropped.
const DEBOUNCE: Duration = Duration::from_millis(500);

/// One sensor's Home Assistant wiring: the row name (matches the decoder and
/// state topic), its discovery config topic + payload, and its state topic.
struct Sensor {
    row: &'static str,
    disc_topic: &'static str,
    disc_payload: &'static str,
    state_topic: &'static str,
}

/// Build one [`Sensor`] entry. `$row` is both the decoder row name and the
/// per-sensor suffix of every topic and id, matching the original hand-written
/// table. `AQ_ID` (from `secrets.env`, default `aq`) prefixes every topic,
/// `uniq_id` and the HA device id, so a second unit only needs a different
/// `AQ_ID` to coexist with the first instead of overwriting its entities. The discovery payload is assembled with `concat!` so it stays a
/// `&'static str` with no runtime formatting.
macro_rules! sensor {
    ($row:literal, $name:literal, $dev_cla:literal, $unit:literal) => {
        Sensor {
            row: $row,
            disc_topic: concat!("homeassistant/sensor/", env!("AQ_ID"), "/", $row, "/config"),
            disc_payload: concat!(
                r#"{"name":""#, $name,
                r#"","uniq_id":""#, env!("AQ_ID"), "_", $row,
                r#"","stat_t":""#, env!("AQ_ID"), "/", $row,
                r#"","dev_cla":""#, $dev_cla,
                r#"","unit_of_meas":""#, $unit,
                r#"","stat_cla":"measurement","dev":{"ids":[""#, env!("AQ_ID"),
                r#""],"name":""#, env!("AQ_NAME"), r#""}}"#,
            ),
            state_topic: concat!(env!("AQ_ID"), "/", $row),
        }
    };
}

// The panel reports PM2.5 (µg/m³), TVOC (ppm per the panel label), CO2 (ppm),
// temperature (°C) and humidity (%). State topics live under `<AQ_ID>/`
// (subscribe to `<AQ_ID>/#`); discovery stays under `homeassistant/` as HA
// requires. All share one device so HA groups them.
const SENSORS: &[Sensor] = &[
    sensor!("pm25", "PM2.5", "pm25", "\u{b5}g/m\u{b3}"),
    sensor!("tvoc", "TVOC", "volatile_organic_compounds_parts", "ppm"),
    sensor!("co2", "CO2", "carbon_dioxide", "ppm"),
    sensor!("temp", "Temperature", "temperature", "\u{b0}C"),
    sensor!("humidity", "Humidity", "humidity", "%"),
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
        // Must exceed PING_INTERVAL with margin, or the socket idle-times-out
        // between pings and closes itself (broker: "connection closed by client").
        sock.set_timeout(Some(Duration::from_secs(MQTT_KEEPALIVE_SECS as u64 * 2)));

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
            .keep_alive(KeepAlive::Seconds(NonZero::new(MQTT_KEEPALIVE_SECS).unwrap()))
            .user_name(MqttString::try_from(HA_USER).unwrap())
            .password(MqttBinary::try_from(HA_TOKEN.as_bytes()).unwrap());

        match client
            .connect(
                sock,
                &connect_opts,
                Some(MqttString::try_from(CLIENT_ID).unwrap()),
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

        // Debounce state. `pending[i]` holds a value that differs from `last[i]`
        // but hasn't been stable long enough to publish yet; `deadline[i]` is when
        // it becomes publishable. Both are cleared once the value is published.
        let mut pending: [heapless::String<16>; SENSORS.len()] = Default::default();
        let mut deadline: [Option<Instant>; SENSORS.len()] = Default::default();

        // Watchdog: the instant since which at least one sensor's last-published
        // value has looked malformed. Evaluated over the whole `last[]` snapshot
        // on every ping tick (not per-update), so a value stuck bad still trips
        // it. Reboots if any sensor stays bad for 5 minutes.
        let mut bad_since: Option<Instant> = None;

        'connected: loop {
            // rust-mqtt is poll-driven: client.poll() reads incoming packets
            // (PINGRESP, broker control traffic). Without it the socket RX backs
            // up and the broker drops us. The ping keeps the connection (and the
            // TCP idle timer) alive when sensor values stall. poll() idles in the
            // cancel-safe poll_header, so losing the select race is fine.
            let next_ping = Timer::after(PING_INTERVAL);
            // Wake when the earliest pending debounce deadline elapses (if any),
            // so we publish the settled value without waiting on the next update.
            let next_debounce = async {
                match deadline.iter().flatten().min() {
                    Some(&d) => Timer::at(d).await,
                    None => core::future::pending().await,
                }
            };
            match select4(
                sub.next_message_pure(),
                next_ping,
                client.poll(),
                next_debounce,
            )
            .await
            {
                Either4::First(update) => {
                    let Some(idx) = SENSORS.iter().position(|s| s.row == update.name) else {
                        continue;
                    };
                    // Stage the value for debounced publishing. If it matches what
                    // we last published, cancel any pending change (the panel
                    // bounced back). Otherwise (re)arm the debounce timer so we
                    // only publish once the digits stop moving.
                    if last[idx] == update.value {
                        pending[idx].clear();
                        deadline[idx] = None;
                    } else if pending[idx] != update.value {
                        pending[idx].clear();
                        let _ = pending[idx].push_str(&update.value);
                        deadline[idx] = Some(Instant::now() + DEBOUNCE);
                    }
                }
                Either4::Second(_) => {
                    // Watchdog: scan the current last-published value of every
                    // sensor. If any looks malformed, arm `bad_since`; once a bad
                    // value has stood for 5 minutes, reboot to reset the decoder.
                    // Any all-clean scan disarms it. Runs on the ping tick so a
                    // value stuck bad (no new updates) is still caught.
                    let any_bad = last.iter().any(|v| is_value_malformed(v));
                    if any_bad {
                        let since = *bad_since.get_or_insert_with(Instant::now);
                        if since.elapsed() >= Duration::from_secs(300) {
                            info!("watchdog: malformed data persisted 5 min, rebooting");
                            unsafe { esp_rom_software_reset_system() };
                        } else {
                            info!("watchdog: malformed data present, armed");
                        }
                    } else if bad_since.take().is_some() {
                        info!("watchdog: data clean again, disarmed");
                    }

                    match client.ping().await {
                        Ok(()) => info!("MQTT ping ok"),
                        Err(e) => {
                            info!("MQTT ping failed: {e:?}, reconnecting");
                            break 'connected;
                        }
                    }
                }
                Either4::Third(result) => {
                    if let Err(e) = result {
                        info!("MQTT poll failed: {e:?}, reconnecting");
                        break 'connected;
                    }
                    // else: drained an incoming packet (e.g. PINGRESP)
                }
                Either4::Fourth(()) => {
                    // A debounce deadline elapsed: publish every pending value
                    // whose timer has expired (a single wake can settle several).
                    let now = Instant::now();
                    for idx in 0..SENSORS.len() {
                        match deadline[idx] {
                            Some(d) if d <= now => {}
                            _ => continue,
                        }

                        let topic = TopicName::new(
                            MqttString::try_from(SENSORS[idx].state_topic).unwrap(),
                        )
                        .unwrap();
                        let opts =
                            PublicationOptions::new(TopicReference::Name(topic)).retain();
                        match client
                            .publish(&opts, Bytes::from(pending[idx].as_bytes()))
                            .await
                        {
                            Ok(_) => {
                                info!(
                                    "MQTT {} = {}",
                                    SENSORS[idx].row,
                                    pending[idx].as_str()
                                );
                                last[idx].clear();
                                let _ = last[idx].push_str(&pending[idx]);
                                pending[idx].clear();
                                deadline[idx] = None;
                            }
                            Err(e) => {
                                info!("MQTT publish failed: {e:?}, reconnecting");
                                break 'connected;
                            }
                        }
                    }
                }
            }
        }
    }
}
