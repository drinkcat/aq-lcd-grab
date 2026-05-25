#![no_std]
#![no_main]

mod capture;
mod wire;

use embassy_executor::Spawner;
use embassy_futures::join::{join, join3};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{BufferedUart, Config as UsartConfig};
use embassy_stm32::usart::BufferedInterruptHandler;
use embassy_stm32::{bind_interrupts, peripherals, Config};
use embedded_io_async::{Read, Write};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use panic_halt as _;

use capture::{Capture, CapturePins};
use wire::{Encoder, HOST_CMD_START, HOST_CMD_STATS, HOST_CMD_STOP, Sink};

bind_interrupts!(struct Irqs {
    USART1 => BufferedInterruptHandler<peripherals::USART1>;
});

/// Outbound byte queue (encoder → UART TX task). 4 KiB holds ~5 ms of
/// peak-rate output at 921600 baud, plenty for the encoder to write a
/// few frames ahead of the wire.
const TX_QUEUE_CAP: usize = 4096;
type TxQueue = Channel<ThreadModeRawMutex, u8, TX_QUEUE_CAP>;
static TX_QUEUE: TxQueue = Channel::new();

/// Commands from RX task to capture task. The protocol mandates an ack
/// for every START/STOP — even when it's a no-op for the current
/// state — so we route each command through a FIFO instead of a
/// one-slot Signal. 8 slots is plenty: commands are rare and the
/// capture task drains them every polling tick.
type CmdQueue = Channel<ThreadModeRawMutex, HostCmd, 8>;
static CMD_QUEUE: CmdQueue = Channel::new();

/// Shared "we are streaming" flag for the LED task. Updated by the
/// capture task on state transitions; only read by the LED.
static STREAMING: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

#[derive(Copy, Clone, PartialEq, Eq)]
enum StreamState {
    Stopped,
    Streaming,
}

#[derive(Copy, Clone, PartialEq, Eq)]
enum HostCmd {
    Start,
    Stop,
    Stats,
}

/// `Sink` that stages each frame into a small scratch buffer, then
/// atomically pushes the whole frame into the TX queue on
/// `commit_frame()`. If the queue can't fit the frame, the whole frame
/// is discarded — never a partial. Essential for wire-format integrity:
/// a half-emitted frame would desync the host's parser.
///
/// Max frame size is bounded by tag=0x01 with n=255 → 2 + 4*255 = 1022 B.
/// Round up to 1024 for headroom.
const SINK_FRAME_MAX: usize = 1024;
struct QueueSink {
    buf: [u8; SINK_FRAME_MAX],
    n: usize,
    overflowed: bool,
    /// Total bytes discarded since boot.
    dropped: u32,
}

impl QueueSink {
    const fn new() -> Self {
        Self {
            buf: [0; SINK_FRAME_MAX],
            n: 0,
            overflowed: false,
            dropped: 0,
        }
    }
}

impl Sink for QueueSink {
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
            self.dropped = self.dropped.saturating_add(frame_size as u32);
            self.overflowed = false;
            return;
        }
        // All-or-nothing: only enqueue when the whole frame fits, so
        // the queue never carries a torn frame.
        if TX_QUEUE.capacity() - TX_QUEUE.len() < frame_size {
            self.dropped = self.dropped.saturating_add(frame_size as u32);
            return;
        }
        for &b in &self.buf[..frame_size] {
            // Pre-checked free slots, so try_send can't fail.
            let _ = TX_QUEUE.try_send(b);
        }
    }
}

/// Capture ring lengths. 1024 half-words = 2 KiB per ring = 4 KiB
/// total. At 667 kHz peak, 1024 samples ≈ 1.5 ms of headroom — enough
/// to ride out a UART stall while we flush a frame.
const RING_LEN: usize = 1024;
static mut PA_BUF: [u16; RING_LEN] = [0; RING_LEN];
static mut PB_BUF: [u16; RING_LEN] = [0; RING_LEN];

/// Drain chunk: how many paired samples we pull from the rings per
/// poll. Small enough to keep latency tight, large enough to amortise
/// the rings's `read()` overhead.
const DRAIN_CHUNK: usize = 128;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // 64 MHz from HSI+PLL, per pcb_spec.md "Clocking". F1 HSI->PLL is
    // hardwired /2 — embassy-stm32 hard-panics if prediv != DIV2.
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = None;
        config.rcc.pll = Some(Pll {
            src: PllSource::HSI,
            prediv: PllPreDiv::DIV2,
            mul: PllMul::MUL16,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2; // APB1 max 36 MHz
        config.rcc.apb2_pre = APBPrescaler::DIV1;
    }
    let p = embassy_stm32::init(config);

    // PC13 LED — slow blink in STOPPED, fast in STREAMING.
    let mut led = Output::new(p.PC13, Level::High, Speed::Low);

    // USART1 @ 921600. We use BufferedUart (interrupt-driven, no DMA)
    // because USART1_RX's default DMA channel is DMA1_CH5 — and we
    // need Ch5 for TIM2_CH1 capture. Interrupt-driven RX is fine: host
    // commands are single-byte and rare. TX is also interrupt-driven;
    // at 921600 baud the ~10 µs ISR cadence is well below the 64 MHz
    // CPU's critical-path budget.
    static mut UART_TX_BUF: [u8; 4096] = [0; 4096];
    static mut UART_RX_BUF: [u8; 64] = [0; 64];
    let (tx_buf, rx_buf) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(UART_TX_BUF),
            &mut *core::ptr::addr_of_mut!(UART_RX_BUF),
        )
    };
    let mut usart_cfg = UsartConfig::default();
    usart_cfg.baudrate = 921600;
    let usart =
        BufferedUart::new(p.USART1, p.PA10, p.PA9, tx_buf, rx_buf, Irqs, usart_cfg).unwrap();
    let (mut tx, mut rx) = usart.split();

    // Capture front-end — TIM1 ETR + 2 DMA rings.
    let (pa_buf, pb_buf) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(PA_BUF),
            &mut *core::ptr::addr_of_mut!(PB_BUF),
        )
    };
    let mut capture = Capture::new(
        p.TIM2,
        CapturePins { wr_etr: p.PA0 },
        p.DMA1_CH5, // TIM2_CH1 (input capture on TI1) -> PA ring
        p.DMA1_CH7, // TIM2_CH2 (input capture on TI1, alt) -> PB ring
        pa_buf,
        pb_buf,
    );

    let mut encoder = Encoder::default();
    let mut sink = QueueSink::new();
    wire::encode_log("aq-lcd-grab stm32 firmware booted, awaiting START", &mut sink);

    // LED task — visual liveness, also signals state.
    let led_fut = async {
        loop {
            let interval = if STREAMING.load(core::sync::atomic::Ordering::Relaxed) {
                100
            } else {
                500
            };
            led.toggle();
            Timer::after_millis(interval).await;
        }
    };

    // UART TX task — drains the byte queue out the wire.
    let tx_fut = async {
        let mut buf = [0u8; 256];
        loop {
            // Block for the first byte, then opportunistically batch.
            buf[0] = TX_QUEUE.receive().await;
            let mut n = 1;
            while n < buf.len() {
                match TX_QUEUE.try_receive() {
                    Ok(b) => {
                        buf[n] = b;
                        n += 1;
                    }
                    Err(_) => break,
                }
            }
            if <_ as Write>::write(&mut tx, &buf[..n]).await.is_err() {
                // UART error is fatal-ish; the host will reset us.
                // Just drop and continue draining so we don't deadlock.
            }
        }
    };

    // UART RX task — single-byte commands from the host.
    let rx_fut = async {
        let mut byte = [0u8; 1];
        loop {
            if <_ as Read>::read(&mut rx, &mut byte).await.is_err() {
                continue;
            }
            let cmd = match byte[0] {
                HOST_CMD_START => HostCmd::Start,
                HOST_CMD_STOP => HostCmd::Stop,
                HOST_CMD_STATS => HostCmd::Stats,
                _ => continue, // unknown command, ignore
            };
            CMD_QUEUE.send(cmd).await;
        }
    };

    // Capture-drain task — pulls samples, feeds encoder, handles state
    // transitions and overrun reporting.
    let mut state = StreamState::Stopped;
    let cap_fut = async {
        let mut pa_buf = [0u16; DRAIN_CHUNK];
        let mut pb_buf = [0u16; DRAIN_CHUNK];
        let mut idle_ticks: u32 = 0;
        loop {
            // Drain all pending host commands. Every START/STOP gets an
            // ack even when it's a no-op for the current state — the
            // host's sync handshake assumes STOP always produces FC.
            while let Ok(cmd) = CMD_QUEUE.try_receive() {
                match cmd {
                    HostCmd::Start => {
                        if state == StreamState::Stopped {
                            encoder.reset();
                            state = StreamState::Streaming;
                            STREAMING.store(true, core::sync::atomic::Ordering::Relaxed);
                        }
                        wire::encode_started(&mut sink);
                    }
                    HostCmd::Stop => {
                        if state == StreamState::Streaming {
                            encoder.flush(&mut sink);
                            state = StreamState::Stopped;
                            STREAMING.store(false, core::sync::atomic::Ordering::Relaxed);
                        }
                        wire::encode_stopped(&mut sink);
                    }
                    HostCmd::Stats => {
                        let mut buf = [0u8; 64];
                        let msg = fmt_stats(
                            &mut buf,
                            sink.dropped,
                            capture.peek_dropped_total(),
                        );
                        wire::encode_log(msg, &mut sink);
                    }
                }
            }

            // Drain rings regardless of state — keeping them empty
            // prevents an overrun marker the moment we start.
            let n = capture.drain(&mut pa_buf, &mut pb_buf);

            if n > 0 && state == StreamState::Streaming {
                for i in 0..n {
                    // pa = GPIOA->IDR low 16, pb = GPIOB->IDR low 16.
                    // Packing into one u32 matches the on-wire LE byte
                    // layout exactly; the host's permute layer
                    // unpacks them per-board.
                    let sample = pa_buf[i] as u32 | (pb_buf[i] as u32) << 16;
                    encoder.feed(sample, &mut sink);
                }
                // Flush block/run on each drain cycle so latency is
                // bounded by the drain period, not by waiting for a
                // 255-sample fill.
                encoder.flush(&mut sink);

                let dropped = capture.take_dropped();
                if dropped > 0 {
                    wire::encode_overrun(dropped, &mut sink);
                }
            } else {
                // Discard accumulated drops in STOPPED mode — they're
                // not interesting.
                let _ = capture.take_dropped();
            }

            // Always yield once per outer iteration. With ETR=PA12
            // floating on a bench rig (no real WR signal), the timer
            // picks up enough noise that `capture.drain` always returns
            // samples and the inner loop never sleeps — without this
            // yield, the LED/RX/TX tasks would never get scheduled.
            embassy_futures::yield_now().await;

            if n == 0 {
                idle_ticks = idle_ticks.wrapping_add(1);
                if idle_ticks.is_multiple_of(2500) {
                    // ~5 s heartbeat. Dump TIM2->CNT plus the NDTR
                    // remaining-transfer counters for the two DMA
                    // channels. If CNT advances between ticks, ETR is
                    // counting WR edges. If NDTR decrements, DMA is
                    // firing.
                    if state == StreamState::Streaming {
                        let cnt = capture.peek_cnt();
                        let (pa_ndtr, pb_ndtr) = capture.peek_dma_ndtr();
                        let mut buf = [0u8; 48];
                        let msg = fmt_idle3(&mut buf, cnt, pa_ndtr, pb_ndtr);
                        wire::encode_log(msg, &mut sink);
                    }
                }
                Timer::after_millis(2).await;
            } else {
                idle_ticks = 0;
            }
        }
    };

    join(join3(led_fut, tx_fut, rx_fut), cap_fut).await;
}

/// Format `"cnt=XXXX pa_ndtr=XXXX pb_ndtr=XXXX"` into `buf` and
/// return a &str. `buf` must be at least 36 bytes.
fn fmt_idle3(buf: &mut [u8], cnt: u16, pa_ndtr: u16, pb_ndtr: u16) -> &str {
    let mut pos = 0;
    pos += write_label_u16(&mut buf[pos..], b"cnt=", cnt);
    buf[pos] = b' '; pos += 1;
    pos += write_label_u16(&mut buf[pos..], b"pa_ndtr=", pa_ndtr);
    buf[pos] = b' '; pos += 1;
    pos += write_label_u16(&mut buf[pos..], b"pb_ndtr=", pb_ndtr);
    core::str::from_utf8(&buf[..pos]).unwrap()
}

fn write_label_u16(buf: &mut [u8], label: &[u8], val: u16) -> usize {
    buf[..label.len()].copy_from_slice(label);
    for i in 0..4 {
        let nibble = (val >> (12 - 4 * i)) & 0xF;
        buf[label.len() + i] = if nibble < 10 {
            b'0' + nibble as u8
        } else {
            b'a' + (nibble - 10) as u8
        };
    }
    label.len() + 4
}

/// Format `"stats: tx_dropped=XXXXXXXX cap_dropped=XXXXXXXX"` into
/// `buf` and return a &str. `buf` must be at least 44 bytes.
fn fmt_stats(buf: &mut [u8], tx_dropped: u32, cap_dropped: u32) -> &str {
    let mut pos = 0;
    let prefix = b"stats: ";
    buf[..prefix.len()].copy_from_slice(prefix);
    pos += prefix.len();
    pos += write_label_u32(&mut buf[pos..], b"tx_dropped=", tx_dropped);
    buf[pos] = b' '; pos += 1;
    pos += write_label_u32(&mut buf[pos..], b"cap_dropped=", cap_dropped);
    core::str::from_utf8(&buf[..pos]).unwrap()
}

fn write_label_u32(buf: &mut [u8], label: &[u8], val: u32) -> usize {
    buf[..label.len()].copy_from_slice(label);
    for i in 0..8 {
        let nibble = (val >> (28 - 4 * i)) & 0xF;
        buf[label.len() + i] = if nibble < 10 {
            b'0' + nibble as u8
        } else {
            b'a' + (nibble - 10) as u8
        };
    }
    label.len() + 8
}

