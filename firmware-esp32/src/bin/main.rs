#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

//! aq-lcd-grab ESP32-C6 gateway.
//!
//! Receives the wire-protocol capture stream over UART from the STM32/Pico
//! bridge, decodes it on-chip (wire → permute → glyph decoder + framebuffer),
//! publishes sensor values to Home Assistant over MQTT, and serves the
//! reconstructed panel image over HTTP.
//!
//! This is the bring-up scaffold: WiFi + DHCP only. The UART/decode, HTTP, and
//! MQTT tasks are added in subsequent steps.

use aq_lcd_grab_esp32::pipeline::{HOST_CMD_START, HOST_CMD_STOP, Pipeline};
use aq_lcd_grab_esp32::{SharedFb, VALUES};
use embassy_executor::Spawner;
use embassy_net::{Runner, StackResources};
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_hal::Async;
use esp_radio::wifi::{ClientConfig, ModeConfig, WifiController, WifiDevice};
use framebuffer::{Framebuffer, Palette4Store, DEFAULT_PALETTE, PIXELS};
use log::{info, warn};
use static_cell::StaticCell;

extern crate alloc;
use alloc::string::ToString as _;

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");

// Default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

#[embassy_executor::task]
async fn wifi_task(mut controller: WifiController<'static>) {
    loop {
        info!("WiFi connecting...");
        match controller.connect_async().await {
            Ok(()) => {
                info!("WiFi connected!");
                controller
                    .wait_for_event(esp_radio::wifi::WifiEvent::StaDisconnected)
                    .await;
                info!("WiFi disconnected, reconnecting...");
            }
            Err(e) => {
                info!("WiFi connect failed: {e:?}, retrying in 5s");
                Timer::after(Duration::from_secs(5)).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

/// Drive the decode pipeline from the bridge UART. Performs the START/STOP
/// handshake, then streams bytes into the wire/glyph/framebuffer pipeline.
#[embassy_executor::task]
async fn uart_task(mut uart: Uart<'static, Async>, fb: &'static SharedFb) {
    let values_pub = VALUES.publisher().unwrap();
    let mut pipeline = Pipeline::new();
    let mut buf = [0u8; 512];

    // Handshake: tell the bridge to (re)start streaming. The bridge replies
    // with STARTED and then frame data. We don't strictly need to drain first
    // since the wire decoder resyncs on a clean frame boundary after START.
    let _ = uart.write_async(&[HOST_CMD_STOP]).await;
    Timer::after(Duration::from_millis(50)).await;
    let _ = uart.write_async(&[HOST_CMD_START]).await;
    info!("uart: sent START to bridge");

    loop {
        match uart.read_async(&mut buf).await {
            Ok(0) => {}
            Ok(n) => {
                if let Err(e) = pipeline.feed(&buf[..n], fb, &values_pub).await {
                    warn!("wire desync ({e:?}), re-syncing");
                    let _ = uart.write_async(&[HOST_CMD_START]).await;
                }
            }
            Err(e) => {
                warn!("uart read error: {e:?}");
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Heap for esp-radio + alloc-using deps (rust-mqtt). The 75 KiB framebuffer
    // and decode buffers are static, not heap. 64 KiB is the most the reclaimed
    // DRAM2 region holds on this config.
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("aq-lcd-grab ESP32-C6 gateway starting");

    // Shared framebuffer: 4bpp into the default palette, backed by a 'static
    // buffer. PIXELS/2 = 76 800 bytes.
    let fb: &'static SharedFb = {
        static FB_BYTES: StaticCell<[u8; PIXELS / 2]> = StaticCell::new();
        let bytes = FB_BYTES.init([0u8; PIXELS / 2]);
        let store = Palette4Store::new(bytes, DEFAULT_PALETTE);
        static FB: StaticCell<SharedFb> = StaticCell::new();
        FB.init(SharedFb::new(Framebuffer::new(store)))
    };

    // Bridge UART (UART1). Board header pins: RX = GPIO17 (receives the
    // bridge's TX), TX = GPIO16 (sends START/STOP back). 921600 8N1 matches the
    // STM32F103 bench rig. (Console logging goes over USB-CDC, so these GPIOs
    // are free for the bridge link.)
    let uart = {
        let cfg = UartConfig::default().with_baudrate(921_600);
        Uart::new(peripherals.UART1, cfg)
            .expect("uart config")
            .with_rx(peripherals.GPIO17)
            .with_tx(peripherals.GPIO16)
            .into_async()
    };

    static RADIO_INIT: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();
    let radio_init =
        RADIO_INIT.init(esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller"));
    let (mut wifi_controller, interfaces) =
        esp_radio::wifi::new(&*radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    wifi_controller
        .set_config(&ModeConfig::Client(
            ClientConfig::default()
                .with_ssid(WIFI_SSID.to_string())
                .with_password(WIFI_PASSWORD.to_string()),
        ))
        .expect("Failed to configure WiFi");
    wifi_controller.start().expect("Failed to start WiFi");

    let seed = {
        let rng = Rng::new();
        (rng.random() as u64) << 32 | rng.random() as u64
    };

    static STACK_RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        interfaces.sta,
        embassy_net::Config::dhcpv4(Default::default()),
        STACK_RESOURCES.init(StackResources::new()),
        seed,
    );

    spawner.spawn(wifi_task(wifi_controller)).unwrap();
    spawner.spawn(net_task(runner)).unwrap();
    // The decode pipeline runs independently of WiFi so capture works even
    // before the network is up.
    spawner.spawn(uart_task(uart, fb)).unwrap();

    stack.wait_config_up().await;
    if let Some(cfg) = stack.config_v4() {
        info!("WiFi ready, IP: {}", cfg.address);
    }

    loop {
        Timer::after(Duration::from_secs(30)).await;
        if let Some(cfg) = stack.config_v4() {
            info!("alive, IP: {}", cfg.address);
        }
    }
}
