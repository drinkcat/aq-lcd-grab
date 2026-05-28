#![no_std]
#![no_main]

mod picotool_reset;
mod pio_capture;

use embassy_executor::Spawner;
use embassy_futures::join::{join, join3};
use embassy_rp::bind_interrupts;
use embassy_rp::dma;
use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::pipe::Pipe;
use embassy_time::{Instant, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcState};
use embassy_usb::{Builder, Config};
use panic_halt as _;

use picotool_reset::PicotoolHandler;
use pio_capture::{CapturePins, RingCapture, Sample as RawSample};
use wire::{Encoder, HOST_CMD_LOG_TEST, HOST_CMD_START, HOST_CMD_STATS, HOST_CMD_STOP, Sink};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
});

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"aq-lcd-grab capture"),
    embassy_rp::binary_info::rp_program_description!(
        c"PIO+DMA capture of 8080 bus -> tagged wire stream (see docs/wire_protocol.md)"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

// PIO ring: 8192 samples × 4 B = 32 KiB (RP2350 max ring_size=15). At
// 200 kHz the ring buys ~40 ms of headroom, easily enough to ride out
// USB stalls.
const RING_LEN: usize = 8192;
#[repr(align(32768))]
struct RingBuf([RawSample; RING_LEN]);
static mut RING_BUF: RingBuf = RingBuf([0; RING_LEN]);

/// Samples we pull off the PIO ring per polling tick. Smaller =
/// tighter latency, larger = less per-tick overhead. 1024 matches the
/// STM32 build's drain chunk.
const DRAIN_CHUNK: usize = 1024;

/// Outbound byte pipe (encoder → USB sender). 4 KiB holds ~60 ms at
/// USB-FS bulk burst rate (≈ 64 kB/s) and ~10 KB worth of typical
/// target bus traffic — plenty to bridge a USB stall.
///
/// We use a byte-oriented `Pipe` rather than a `Channel<u8>` because the
/// USB sender needs bulk reads to fill 64-byte packets; pulling one byte
/// per `try_receive` capped throughput at ~15 kB/s (1 packet per USB
/// frame, mostly short).
const TX_PIPE_CAP: usize = 4096;
static TX_PIPE: Pipe<ThreadModeRawMutex, TX_PIPE_CAP> = Pipe::new();

/// Commands from the RX task to the capture task. The protocol mandates
/// an ack for every START/STOP — even when the command is a no-op for
/// the current state — so we send each command through a FIFO instead
/// of a one-slot Signal. 8 slots is plenty: a host won't issue commands
/// fast enough to outrun a polling tick.
type CmdQueue = Channel<ThreadModeRawMutex, HostCmd, 8>;
static CMD_QUEUE: CmdQueue = Channel::new();

#[derive(Copy, Clone, PartialEq, Eq)]
enum StreamState {
    Stopped,
    Streaming,
}

#[derive(Copy, Clone, PartialEq, Eq)]
enum HostCmd {
    Start,
    Stop,
    LogTest,
    Stats,
}

/// `Sink` that writes wire bytes straight into the TX pipe. When the
/// pipe is full a byte is dropped and counted — the host tolerates a
/// torn frame by resyncing (STOP → drain → START) on the next bad
/// parse, so we don't stage whole frames. Backpressure shows up at the
/// capture layer instead: a full pipe slows the drain loop and the PIO
/// ring overruns, which `capture.take_dropped()` already reports.
struct PipeSink {
    /// Bytes dropped since boot because the TX pipe was full. STATS
    /// surfaces this as a TX-path health signal.
    dropped: u32,
    /// Cumulative bytes successfully enqueued to TX_PIPE since boot.
    /// Wraps every ~4 GB; the capture task diffs against a per-tick
    /// baseline for the compression-ratio telemetry. A window's single
    /// TICK frame is emitted *after* its `bytes_out` delta is read and
    /// the baseline re-captured, so TICK bytes never inflate a window.
    bytes_out: u32,
}

impl PipeSink {
    const fn new() -> Self {
        Self {
            dropped: 0,
            bytes_out: 0,
        }
    }
}

impl Sink for PipeSink {
    fn push(&mut self, b: u8) -> bool {
        match TX_PIPE.try_write(&[b]) {
            Ok(_) => {
                self.bytes_out = self.bytes_out.wrapping_add(1);
                true
            }
            Err(_) => {
                self.dropped = self.dropped.saturating_add(1);
                false
            }
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let driver = Driver::new(p.USB, Irqs);

    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("aq-lcd-grab");
    config.product = Some("Capture board (Pico 2 W)");
    config.serial_number = Some("aq-lcd-grab");
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    static mut CONFIG_DESC: [u8; 256] = [0; 256];
    static mut BOS_DESC: [u8; 256] = [0; 256];
    static mut MSOS_DESC: [u8; 256] = [0; 256];
    static mut CONTROL_BUF: [u8; 64] = [0; 64];
    static CDC_STATE: static_cell::StaticCell<CdcState> = static_cell::StaticCell::new();
    static PICOTOOL_HANDLER: static_cell::StaticCell<PicotoolHandler> =
        static_cell::StaticCell::new();

    let cdc_state = CDC_STATE.init(CdcState::new());
    let picotool_handler = PICOTOOL_HANDLER.init(PicotoolHandler::new());

    // SAFETY: each static-mut buffer is borrowed exactly once.
    let (config_desc, bos_desc, msos_desc, control_buf) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(CONFIG_DESC),
            &mut *core::ptr::addr_of_mut!(BOS_DESC),
            &mut *core::ptr::addr_of_mut!(MSOS_DESC),
            &mut *core::ptr::addr_of_mut!(CONTROL_BUF),
        )
    };

    let mut builder = Builder::new(
        driver,
        config,
        config_desc,
        bos_desc,
        msos_desc,
        control_buf,
    );

    let cdc_class = CdcAcmClass::new(&mut builder, cdc_state, 64);

    let iface_num = {
        let mut func = builder.function(0xFF, 0x00, 0x01);
        let mut iface = func.interface();
        let num = iface.interface_number();
        let _alt = iface.alt_setting(0xFF, 0x00, 0x01, None);
        num
    };
    picotool_handler.set_interface(iface_num);
    builder.handler(picotool_handler);

    let mut usb = builder.build();

    let pio = Pio::new(p.PIO0, Irqs);
    // SAFETY: RING_BUF is only borrowed here, exactly once, for the
    // remainder of the program.
    let ring_slice: &'static mut [RawSample] = unsafe {
        let ptr = core::ptr::addr_of_mut!(RING_BUF.0) as *mut RawSample;
        core::slice::from_raw_parts_mut(ptr, RING_LEN)
    };
    let mut capture = RingCapture::new(
        pio,
        p.DMA_CH0,
        CapturePins {
            db0: p.PIN_0,
            db1: p.PIN_1,
            db2: p.PIN_2,
            db3: p.PIN_3,
            db4: p.PIN_4,
            db5: p.PIN_5,
            db6: p.PIN_6,
            db7: p.PIN_7,
            db8: p.PIN_8,
            db9: p.PIN_9,
            db10: p.PIN_10,
            db11: p.PIN_11,
            db12: p.PIN_12,
            db13: p.PIN_13,
            db14: p.PIN_14,
            db15: p.PIN_15,
            cs: p.PIN_16,
            dc: p.PIN_17,
            wr: p.PIN_18,
        },
        Irqs,
        ring_slice,
    );

    let (mut sender, mut receiver) = cdc_class.split();

    let mut encoder = Encoder::default();
    let mut sink = PipeSink::new();
    encoder.log("aq-lcd-grab pico firmware booted, awaiting START", &mut sink);

    let usb_fut = usb.run();

    // USB TX task — drains the byte pipe out the CDC sender.
    //
    // The RP2350 USB is Full-Speed (1 SOF/ms). The Linux cdc_acm host
    // driver typically issues one IN URB per USB frame, so each
    // `write_packet` we send costs one frame's worth of latency
    // regardless of packet size. To approach the FS bulk ceiling
    // (≈ 64 kB/s), we must fill every packet to the 64-byte max.
    //
    // Strategy: wake on the first byte (latency floor for sparse
    // traffic), then wait *briefly* for the packet to fill before
    // shipping. The wait is bounded so interactive acks / log lines
    // still flush within a few ms.
    let tx_fut = async {
        use embassy_futures::select::{select, Either};
        sender.wait_connection().await;
        let mut buf = [0u8; 64];
        loop {
            // Block until at least one byte lands.
            let mut n = TX_PIPE.read(&mut buf).await;
            // Top up the packet, waiting up to ~1 USB frame for more
            // bytes. This lets a steady producer at modest rate (~10s
            // kB/s) fill 64-byte packets instead of dribbling out a
            // packet per byte.
            while n < buf.len() {
                let tail = &mut buf[n..];
                match select(TX_PIPE.read(tail), Timer::after_millis(2)).await {
                    Either::First(extra) => n += extra,
                    Either::Second(_) => break,
                }
            }
            let full_packet = n == buf.len();
            if sender.write_packet(&buf[..n]).await.is_err() {
                sender.wait_connection().await;
                continue;
            }
            if full_packet && TX_PIPE.is_empty() {
                if sender.write_packet(&[]).await.is_err() {
                    sender.wait_connection().await;
                }
            }
        }
    };

    // USB RX task — single-byte commands from the host. The host's
    // sync protocol sends single bytes (`0x01..=0x04`); we still loop
    // over `read_packet`'s buffer in case a future host bundles
    // commands.
    let rx_fut = async {
        let mut buf = [0u8; 64];
        loop {
            let n = match receiver.read_packet(&mut buf).await {
                Ok(n) => n,
                Err(_) => {
                    receiver.wait_connection().await;
                    continue;
                }
            };
            for &b in &buf[..n] {
                let cmd = match b {
                    HOST_CMD_START => HostCmd::Start,
                    HOST_CMD_STOP => HostCmd::Stop,
                    HOST_CMD_LOG_TEST => HostCmd::LogTest,
                    HOST_CMD_STATS => HostCmd::Stats,
                    _ => continue, // unknown command, ignore
                };
                // Block here if the queue is full — the capture task
                // drains it on every poll, so backpressure is benign.
                CMD_QUEUE.send(cmd).await;
            }
        }
    };

    // Capture-drain task — pulls samples, feeds encoder, handles state
    // transitions and overrun reporting.
    let mut state = StreamState::Stopped;
    let cap_fut = async {
        let mut chunk = [0u32; DRAIN_CHUNK];
        let mut idle_ticks: u32 = 0;
        // TICK rate-limit: target ~100 TICKs/sec regardless of drain
        // cadence. Without this, sustained bursts produce one TICK per
        // drain (~500/s of useless telemetry) and a long inner-drain
        // pass produces a single TICK looking deceptively like a gap.
        const TICK_INTERVAL_US: u64 = 10_000;
        let mut last_tick = Instant::now();
        let mut tick_drained: u64 = 0;
        let mut tick_t0 = last_tick;
        let mut tick_bytes_baseline: u32 = 0;
        loop {
            // Drain all pending host commands. Every START/STOP gets an
            // ack even when it's a no-op for the current state — the
            // protocol guarantees this so the host can resync by
            // sending STOP unconditionally.
            while let Ok(cmd) = CMD_QUEUE.try_receive() {
                match cmd {
                    HostCmd::Start => {
                        if state == StreamState::Stopped {
                            encoder.reset();
                            state = StreamState::Streaming;
                        }
                        encoder.started(&mut sink);
                    }
                    HostCmd::Stop => {
                        if state == StreamState::Streaming {
                            encoder.flush(&mut sink);
                            state = StreamState::Stopped;
                        }
                        encoder.stopped(&mut sink);
                    }
                    HostCmd::LogTest => encoder.log("ping", &mut sink),
                    HostCmd::Stats => {
                        let mut msg = heapless::String::<64>::new();
                        use core::fmt::Write as _;
                        let _ = write!(
                            msg,
                            "stats: tx_dropped={} cap_dropped={}",
                            sink.dropped,
                            capture.peek_dropped(),
                        );
                        encoder.log(&msg, &mut sink);
                    }
                }
            }

            // Drain the PIO ring even when STOPPED — keeps it empty so
            // we don't trigger an overrun the moment we transition into
            // STREAMING.
            let mut total = 0usize;
            loop {
                let n = capture.drain(&mut chunk);
                if n == 0 {
                    break;
                }
                total += n;
                if state == StreamState::Streaming {
                    for &raw in &chunk[..n] {
                        // PIO packs `{cs, dc, db15..db0}` (low 16 bits =
                        // data, bit 16 = CS, bit 17 = DC; see
                        // `pio_capture.rs`). The wire protocol carries
                        // the sample as a single LE u32 = `pa | pb<<16`.
                        // For the Pico the natural mapping is:
                        //   pa (low 16) = DB0..DB15 (logical order)
                        //   pb (hi 16)  = bit 0=CS, bit 1=DC, others 0
                        // Mask the noise bits above 17 so spurious
                        // unused-pin flips don't break RLE runs.
                        let sample = raw & 0x0003_FFFF;
                        encoder.feed(sample, &mut sink);
                    }
                }
            }

            if total > 0 && state == StreamState::Streaming {
                // Per-drain transport flush. BLOCK samples already
                // streamed to the sink during `feed` — only a RUN in
                // progress is held in encoder state, and we leave it
                // alive so a uniform fill spanning many drains merges
                // into one big RUN instead of one small RUN per drain.
                // TODO: we only want to force-flush when the bus is
                // idle, not on every drain.
                sink.flush();
            }

            // Tick aggregation: sum drains across the window so a
            // single 50 ms TICK covers however many drain iterations
            // happened in between. Reset on emit.
            if state == StreamState::Streaming {
                tick_drained = tick_drained.saturating_add(total as u64);
                let now = Instant::now();
                if now.duration_since(last_tick).as_micros() >= TICK_INTERVAL_US {
                    let n_pending = capture.available().min(u16::MAX as u32) as u16;
                    let n_drained = tick_drained.min(u16::MAX as u64) as u16;
                    let bytes_now = sink.bytes_out;
                    let bytes_delta = bytes_now.wrapping_sub(tick_bytes_baseline);
                    // Skip pure-idle windows: nothing arrived, nothing
                    // queued, nothing shipped. Keeps the wire silent
                    // when the bus is genuinely quiet; the next
                    // non-zero TICK covers the full idle gap via
                    // dt_us, so no signal is lost.
                    if n_drained > 0 || n_pending > 0 || bytes_delta > 0 {
                        let t_us = tick_t0.as_micros() as u32;
                        let dt_us = now
                            .duration_since(tick_t0)
                            .as_micros()
                            .min(u16::MAX as u64) as u16;
                        encoder.tick(t_us, dt_us, n_drained, n_pending, bytes_delta, &mut sink);
                    }
                    last_tick = now;
                    tick_t0 = now;
                    tick_drained = 0;
                    // Re-baseline *after* the TICK so its own bytes fall
                    // outside the next window's delta.
                    tick_bytes_baseline = sink.bytes_out;
                }
            } else {
                // Keep the window anchored to the current time while
                // STOPPED so the first post-START TICK reports a
                // ~10 ms window, not the entire STOPPED interval.
                tick_t0 = Instant::now();
                last_tick = tick_t0;
                tick_drained = 0;
                tick_bytes_baseline = sink.bytes_out;
            }

            // Overrun reporting: sample-accurate PIO-ring drops (capture
            // overran the DMA buffer). TX-pipe byte drops are a separate,
            // non-sample-exact signal surfaced via STATS, not folded in
            // here.
            if state == StreamState::Streaming {
                let from_capture = capture.take_dropped();
                if from_capture > 0 {
                    encoder.overrun(from_capture, &mut sink);
                }
            }

            // Always yield once per outer iteration. Under sustained
            // capture, the inner `capture.drain` loop never returns
            // zero, so without this yield the TX and RX tasks would
            // never get scheduled — the pipe would fill and bytes
            // would start getting dropped at `push`.
            embassy_futures::yield_now().await;

            if total == 0 {
                if idle_ticks == 0 && state == StreamState::Streaming {
                    // First idle tick after activity: flush any
                    // RUN held across drain boundaries so the host
                    // sees the trailing run within ~2 ms of quiet,
                    // not after the next sample change (which may
                    // never come).
                    encoder.flush(&mut sink);
                }
                idle_ticks = idle_ticks.wrapping_add(1);
                if idle_ticks.is_multiple_of(2500) && state == StreamState::Streaming {
                    // ~5 s heartbeat while streaming idle.
                    encoder.log("idle", &mut sink);
                }
                Timer::after_millis(2).await;
            } else {
                idle_ticks = 0;
            }
        }
    };

    join(join3(usb_fut, tx_fut, rx_fut), cap_fut).await;
}
