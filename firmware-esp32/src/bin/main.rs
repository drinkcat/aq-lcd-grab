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

use aq_lcd_grab_esp32::pipeline::Pipeline;
use aq_lcd_grab_esp32::{SharedFb, VALUES};
use wire::{HOST_CMD_START, HOST_CMD_STOP};
use embassy_executor::Spawner;
use embassy_net::{Runner, StackResources};
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::dma_rx_stream_buffer;
use esp_hal::uart::uhci::{RxConfig as UhciRxConfig, Uhci, UhciRx, UhciTx};
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

/// Drive the decode pipeline from the bridge UART over DMA (UHCI).
///
/// The STM32 bridge streams continuously at 921600; the plain RX FIFO (128 B)
/// can't keep up and overflows. UHCI runs a free-running DMA ring that the CPU
/// drains at its own pace — the same "DMA fills, poll the tail" pattern the
/// capture firmware uses.
///
/// The STM32 boots in Stopped state and only streams after START, so we arm the
/// DMA ring first (to catch the reply), then send START via the UHCI-configured
/// TX. The wire decoder resyncs on the first clean frame boundary.
#[embassy_executor::task]
async fn uart_task(
    uhci_rx: UhciRx<'static, Async>,
    mut uhci_tx: UhciTx<'static, Async>,
    fb: &'static SharedFb,
) {
    let values_pub = VALUES.publisher().unwrap();
    let mut pipeline = Pipeline::new();
    let mut scratch = [0u8; 1024];

    // 16 KiB DMA ring in 2 KiB chunks → ~180 ms of headroom at 921600.
    let stream_buf = dma_rx_stream_buffer!(16 * 1024, 2048);
    let mut transfer = match uhci_rx.read(stream_buf) {
        Ok(t) => t,
        Err((e, _rx, _buf)) => {
            warn!("uhci read start failed: {e:?}");
            return;
        }
    };
    info!("uart: UHCI DMA capture started");

    // Send STOP then START over the (UHCI-configured) UART TX. uart_tx is a
    // normal UART TX — fine for a couple of handshake bytes, no DMA needed.
    let _ = uhci_tx.uart_tx.write_async(&[HOST_CMD_STOP]).await;
    Timer::after(Duration::from_millis(20)).await;
    let _ = uhci_tx.uart_tx.write_async(&[HOST_CMD_START]).await;
    let _ = uhci_tx.uart_tx.flush_async().await;
    info!("uart: sent START to bridge");

    let mut total: u64 = 0;
    let mut last_report = embassy_time::Instant::now();
    loop {
        let avail = transfer.available_bytes();
        if avail == 0 {
            // Nothing yet — yield briefly. At line rate a 2 ms nap still leaves
            // the 16 KiB ring far from full.
            Timer::after(Duration::from_millis(2)).await;
            // Heartbeat so we can tell whether any bytes are arriving at all.
            if last_report.elapsed() >= Duration::from_secs(5) {
                info!("uart: rx total={total} bytes");
                last_report = embassy_time::Instant::now();
            }
            continue;
        }
        let n = transfer.pop(&mut scratch);
        total += n as u64;
        if last_report.elapsed() >= Duration::from_secs(5) {
            info!("uart: rx total={total} bytes");
            last_report = embassy_time::Instant::now();
        }
        if let Err(e) = pipeline.feed(&scratch[..n], fb, &values_pub).await {
            warn!("wire desync ({e:?})");
        }
    }
}

/// Number of concurrent HTTP connection handlers. A browser opens the page and
/// the image on separate connections (and may pipeline refreshes), so a single
/// listener leaves gaps where the next connect is refused. A small pool keeps a
/// listener available.
const HTTP_WORKERS: usize = 3;

/// Serve the reconstructed panel framebuffer over HTTP on port 80.
///
/// Hand-rolled (no web framework): `GET /` returns a tiny auto-refreshing HTML
/// page; `GET /fb.bmp` streams the framebuffer as a 24-bit BMP, generated on
/// the fly in chunks so we never buffer the whole ~460 KiB image.
#[embassy_executor::task(pool_size = HTTP_WORKERS)]
async fn http_task(stack: embassy_net::Stack<'static>, fb: &'static SharedFb, worker: usize) {
    use embassy_net::tcp::TcpSocket;
    use static_cell::ConstStaticCell;

    // Per-worker socket buffers.
    static RX: [ConstStaticCell<[u8; 1024]>; HTTP_WORKERS] =
        [const { ConstStaticCell::new([0; 1024]) }; HTTP_WORKERS];
    static TX: [ConstStaticCell<[u8; 2048]>; HTTP_WORKERS] =
        [const { ConstStaticCell::new([0; 2048]) }; HTTP_WORKERS];
    let rx = RX[worker].take();
    let tx = TX[worker].take();

    loop {
        let mut sock = TcpSocket::new(stack, rx, tx);
        sock.set_timeout(Some(Duration::from_secs(10)));

        if let Err(e) = sock.accept(80).await {
            warn!("http accept failed: {e:?}");
            Timer::after(Duration::from_millis(50)).await;
            continue;
        }

        // Read the request line (we only care about the path). One read of the
        // header is enough for a simple GET.
        let mut req = [0u8; 256];
        let n = sock.read(&mut req).await.unwrap_or(0);
        let path = parse_get_path(&req[..n]);
        info!("http: {} bytes, path={:?}", n, path.map(core::str::from_utf8));

        match path {
            Some(b"/fb.bmp") => serve_bmp(&mut sock, fb).await,
            _ => serve_index(&mut sock).await,
        }

        // Half-close: flush, signal FIN, and wait for the peer to close (or the
        // 10 s timeout) so all data is delivered before the socket is reused.
        let _ = sock.flush().await;
        sock.close();
        let mut drain = [0u8; 64];
        while let Ok(n) = sock.read(&mut drain).await {
            if n == 0 {
                break;
            }
        }
    }
}

/// Extract the request-target from a `GET <path> HTTP/1.x` request line.
fn parse_get_path(req: &[u8]) -> Option<&[u8]> {
    let rest = req.strip_prefix(b"GET ")?;
    let end = rest.iter().position(|&b| b == b' ')?;
    Some(&rest[..end])
}

async fn serve_index(sock: &mut embassy_net::tcp::TcpSocket<'_>) {
    const BODY: &[u8] = b"<!doctype html><meta charset=utf-8><title>aq-lcd</title>\
<style>body{background:#111;margin:0;display:grid;place-items:center;height:100vh}\
img{image-rendering:pixelated;height:96vh}</style>\
<img src=/fb.bmp><script>setInterval(()=>{document.images[0].src='/fb.bmp?'+Date.now()},1000)</script>";
    let _ = write_all(sock, b"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n").await;
    let _ = write_all(sock, BODY).await;
    let _ = sock.flush().await;
}

async fn serve_bmp(sock: &mut embassy_net::tcp::TcpSocket<'_>, fb: &SharedFb) {
    let _ = write_all(
        sock,
        b"HTTP/1.0 200 OK\r\nContent-Type: image/bmp\r\nConnection: close\r\n\r\n",
    )
    .await;

    let fb = fb.lock().await;
    let header = framebuffer::bmp_header();
    if write_all(sock, &header).await.is_err() {
        return;
    }
    // Stream pixel data in chunks; 256 pixels = 768 bytes per write.
    const CHUNK_PX: usize = 256;
    let mut chunk = [0u8; CHUNK_PX * 3];
    let mut start = 0;
    while start < framebuffer::PIXELS {
        let count = CHUNK_PX.min(framebuffer::PIXELS - start);
        let n = framebuffer::bmp_pixels_bgr(&fb, start, count, &mut chunk);
        if write_all(sock, &chunk[..n]).await.is_err() {
            return;
        }
        start += count;
    }
    let _ = sock.flush().await;
}

/// Write the whole buffer, looping over partial writes.
async fn write_all(
    sock: &mut embassy_net::tcp::TcpSocket<'_>,
    mut buf: &[u8],
) -> Result<(), embassy_net::tcp::Error> {
    while !buf.is_empty() {
        let n = sock.write(buf).await?;
        if n == 0 {
            return Err(embassy_net::tcp::Error::ConnectionReset);
        }
        buf = &buf[n..];
    }
    Ok(())
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

    // Bridge UART (UART1) on GPIO17 (RX) / GPIO16 (TX), 921600 8N1.
    //
    // On the ESP32-C6-DevKitC-1 these are U0RXD/U0TXD, wired to the onboard
    // CH343 USB-to-UART bridge that enumerates as the "UART" USB port (VID
    // 1a86). So the host can feed the capture straight into this port — no
    // extra wiring — while the console/logs stay on the native USB-Serial-JTAG
    // port (VID 303a). Off-DevKit, wire the bridge's TX → GPIO17, GND → GND
    // (and GPIO16 → bridge RX for START/STOP).
    let (uhci_rx, uhci_tx) = {
        let cfg = UartConfig::default().with_baudrate(921_600);
        let uart = Uart::new(peripherals.UART1, cfg)
            .expect("uart config")
            .with_rx(peripherals.GPIO17)
            .with_tx(peripherals.GPIO16);

        // UART-over-DMA. chunk_limit must stay ≤ the DMA chunk size (2048).
        let mut uhci = Uhci::new(uart, peripherals.UHCI0, peripherals.DMA_CH0).into_async();
        uhci.set_uart_config(&UartConfig::default().with_baudrate(921_600))
            .expect("uhci uart config");
        let (mut rx, tx) = uhci.split();
        rx.apply_config(&UhciRxConfig::default().with_chunk_limit(1024))
            .expect("uhci rx config");
        (rx, tx)
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

    // Socket slots: HTTP_WORKERS listeners + DHCP + DNS + MQTT headroom.
    static STACK_RESOURCES: StaticCell<StackResources<6>> = StaticCell::new();
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
    spawner.spawn(uart_task(uhci_rx, uhci_tx, fb)).unwrap();

    stack.wait_config_up().await;
    if let Some(cfg) = stack.config_v4() {
        info!("WiFi ready, IP: {} — http://{}/", cfg.address, cfg.address.address());
    }

    for w in 0..HTTP_WORKERS {
        spawner.spawn(http_task(stack, fb, w)).unwrap();
    }

    loop {
        Timer::after(Duration::from_secs(30)).await;
        if let Some(cfg) = stack.config_v4() {
            info!("alive, IP: {}", cfg.address);
        }
    }
}
