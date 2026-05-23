#![no_std]
#![no_main]

mod picotool_reset;
mod pio_capture;
mod wire;

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
use embassy_time::Timer;
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

/// `Sink` that stages each frame into a small scratch buffer, then
/// atomically pushes the whole frame into the TX pipe on
/// `commit_frame()`. If the pipe can't fit the frame, the whole frame
/// is discarded — never a partial. This is essential for wire-format
/// integrity: a half-emitted frame would desync the host's parser.
///
/// Max frame size is bounded by tag=0x01 with n=255 → 2 + 4*255 = 1022 B.
/// Round up to 1024 for headroom.
const SINK_FRAME_MAX: usize = 1024;
struct PipeSink {
    buf: [u8; SINK_FRAME_MAX],
    n: usize,
    /// True if any push in the current frame overflowed the scratch.
    overflowed: bool,
    /// Bytes discarded since boot — sum of dropped frame sizes. STATS
    /// surfaces this; it isn't strictly needed for wire integrity.
    dropped: u32,
    /// Samples (= WR edges) lost to dropped BLOCK/RUN frames, *not yet*
    /// reported to the host. The capture task drains this into a
    /// `tag=0xFD` overrun frame at the next opportunity so the host
    /// knows a gap exists. Cleared on every successful drain.
    pending_dropped_samples: u32,
}

impl PipeSink {
    const fn new() -> Self {
        Self {
            buf: [0; SINK_FRAME_MAX],
            n: 0,
            overflowed: false,
            dropped: 0,
            pending_dropped_samples: 0,
        }
    }

    /// Pull and reset the accumulated count of samples lost to TX-pipe
    /// drops. The capture task calls this to fold pipe drops into the
    /// same overrun frame it already emits for PIO-ring drops.
    fn take_dropped_samples(&mut self) -> u32 {
        core::mem::replace(&mut self.pending_dropped_samples, 0)
    }

    /// Account a dropped frame. Looks at the staged tag/n to decide
    /// how many samples (WR edges) the dropped frame would have
    /// represented; non-sample frames (log, overrun, acks) count 0.
    fn account_dropped(&mut self, frame_size: usize) {
        self.dropped = self.dropped.saturating_add(frame_size as u32);
        let samples = match self.buf.first().copied() {
            Some(wire::TAG_BLOCK) | Some(wire::TAG_RUN) if self.buf.len() >= 2 => {
                self.buf[1] as u32
            }
            _ => 0,
        };
        self.pending_dropped_samples = self.pending_dropped_samples.saturating_add(samples);
    }
}

impl Sink for PipeSink {
    fn push(&mut self, b: u8) -> bool {
        if self.n < self.buf.len() {
            self.buf[self.n] = b;
            self.n += 1;
            true
        } else {
            self.overflowed = true;
            false
        }
    }

    fn commit_frame(&mut self) {
        let frame_size = self.n;
        self.n = 0;
        if self.overflowed {
            // Scratch overflowed — encoder bug or a frame larger than
            // SINK_FRAME_MAX. Either way the frame is unusable.
            self.account_dropped(frame_size);
            self.overflowed = false;
            return;
        }
        // All-or-nothing: only enqueue when the whole frame fits, so the
        // pipe never carries a torn frame.
        if TX_PIPE.free_capacity() < frame_size {
            self.account_dropped(frame_size);
            return;
        }
        // `try_write` returns the size of the largest *contiguous* free
        // region, which can be smaller than `free_capacity()` near the
        // ring's wrap point. Loop until the whole frame is in. We hold
        // the only writer, so progress is guaranteed — each call
        // returns ≥ 1 byte until the frame is fully buffered.
        let mut off = 0;
        while off < frame_size {
            match TX_PIPE.try_write(&self.buf[off..frame_size]) {
                Ok(n) => off += n,
                Err(_) => unreachable!("free_capacity was pre-checked"),
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
            dc: p.PIN_16,
            cs: p.PIN_17,
            wr: p.PIN_18,
        },
        Irqs,
        ring_slice,
    );

    let (mut sender, mut receiver) = cdc_class.split();

    let mut encoder = Encoder::default();
    let mut sink = PipeSink::new();
    wire::encode_log("aq-lcd-grab pico firmware booted, awaiting START", &mut sink);

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
                        wire::encode_started(&mut sink);
                    }
                    HostCmd::Stop => {
                        if state == StreamState::Streaming {
                            encoder.flush(&mut sink);
                            state = StreamState::Stopped;
                        }
                        wire::encode_stopped(&mut sink);
                    }
                    HostCmd::LogTest => wire::encode_log("ping", &mut sink),
                    HostCmd::Stats => {
                        let mut msg = heapless::String::<64>::new();
                        use core::fmt::Write as _;
                        let _ = write!(
                            msg,
                            "stats: tx_dropped={} cap_dropped={}",
                            sink.dropped,
                            capture.peek_dropped(),
                        );
                        wire::encode_log(&msg, &mut sink);
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
                // Flush per drain tick so latency is bounded by the
                // drain period, not by waiting for a 255-sample fill.
                encoder.flush(&mut sink);
            }

            // Overrun reporting. Combine PIO-ring drops (capture
            // overran the DMA buffer) and TX-pipe drops (encoder
            // produced frames faster than USB could ship them) into a
            // single tag=0xFD frame — both manifest the same way on
            // the host: a gap of N missing WR edges in the stream.
            if state == StreamState::Streaming {
                let from_capture = capture.take_dropped();
                let from_pipe = sink.take_dropped_samples();
                let total_dropped = from_capture.saturating_add(from_pipe);
                if total_dropped > 0 {
                    wire::encode_overrun(total_dropped, &mut sink);
                }
            }

            // Always yield once per outer iteration. Under sustained
            // capture, the inner `capture.drain` loop never returns
            // zero, so without this yield the TX and RX tasks would
            // never get scheduled — the pipe would fill and frames
            // would start getting dropped at `commit_frame`.
            embassy_futures::yield_now().await;

            if total == 0 {
                idle_ticks = idle_ticks.wrapping_add(1);
                if idle_ticks.is_multiple_of(2500) && state == StreamState::Streaming {
                    // ~5 s heartbeat while streaming idle.
                    wire::encode_log("idle", &mut sink);
                }
                Timer::after_millis(2).await;
            } else {
                idle_ticks = 0;
            }
        }
    };

    join(join3(usb_fut, tx_fut, rx_fut), cap_fut).await;
}
