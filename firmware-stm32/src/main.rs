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
use embassy_sync::pipe::Pipe;
use embassy_time::Timer;
use panic_halt as _;

use capture::{Capture, CapturePins};
use wire::{Encoder, HOST_CMD_START, HOST_CMD_STATS, HOST_CMD_STOP, Sink};

bind_interrupts!(struct Irqs {
    USART1 => BufferedInterruptHandler<peripherals::USART1>;
});

/// Outbound byte pipe (encoder → UART TX task). 2 KiB holds ~22 ms
/// at the 921600-baud drain rate (≈92 kB/s) — plenty for the encoder
/// to stage many small frames ahead of the wire.
///
/// `Pipe<u8>` instead of `Channel<u8>`: lets the sink push whole
/// frames atomically via `try_write_all` (no per-byte locking) and
/// lets the TX task bulk-read into a stack scratch in one shot.
const TX_PIPE_CAP: usize = 2048;
static TX_PIPE: Pipe<ThreadModeRawMutex, TX_PIPE_CAP> = Pipe::new();

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
/// Max frame size: with `wire::MAX_BLOCK = 16`, tag=0x01 with n=16
/// → 2 + 4*16 = 66 B for data frames. LOG frames (tag=0xFE) can be
/// up to 3 + 256 = 259 B for the longest log message. Round up to
/// 264 for headroom — log frames are rare.
const SINK_FRAME_MAX: usize = 264;
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
        // the pipe never carries a torn frame.
        if TX_PIPE.free_capacity() < frame_size {
            self.dropped = self.dropped.saturating_add(frame_size as u32);
            return;
        }
        // `try_write` can short-write near the ring's wrap point
        // (returns the contiguous free region size). Loop until done —
        // free_capacity was pre-checked so progress is guaranteed.
        let mut off = 0;
        while off < frame_size {
            match TX_PIPE.try_write(&self.buf[off..frame_size]) {
                Ok(n) => off += n,
                Err(_) => unreachable!("free_capacity was pre-checked"),
            }
        }
    }
}

/// Capture ring lengths. 2048 half-words = 4 KiB per ring = 8 KiB
/// total. At 667 kHz peak, 2048 samples ≈ 3 ms of headroom — twice
/// the original 1.5 ms and enough to ride out the drain-thread
/// starvation we were hitting. Could go higher but F103C8 only has
/// 20 KiB SRAM and we also need TX_PIPE + UART buffers.
const RING_LEN: usize = 2048;
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
    static mut UART_TX_BUF: [u8; 1024] = [0; 1024];
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

    // UART TX task — drains TX_PIPE out the wire.
    //
    // `Pipe::read` returns whatever's buffered, up to the slice size.
    // We loop the BufferedUart `write` because it can short-write
    // (returns just what fit in the interrupt-side TX ring);
    // without the inner loop bytes would silently disappear and
    // the host parser would desync.
    let tx_fut = async {
        let mut buf = [0u8; 64];
        loop {
            let n = TX_PIPE.read(&mut buf).await;
            let mut off = 0;
            while off < n {
                match <_ as Write>::write(&mut tx, &buf[off..n]).await {
                    Ok(0) => break, // shouldn't happen, but don't spin
                    Ok(w) => off += w,
                    Err(_) => break, // UART error; host will reset us
                }
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
        // Auto-emit a STATS log line every ~5 s while streaming, so
        // the host's run.log carries a trail of tx_dropped /
        // cap_dropped counters even under sustained traffic (where
        // the idle-tick heartbeat below never fires).
        let mut last_stats = embassy_time::Instant::now();
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

            // Auto-STATS heartbeat every ~5 s while streaming.
            if state == StreamState::Streaming
                && last_stats.elapsed() >= embassy_time::Duration::from_secs(5)
            {
                let mut buf = [0u8; 64];
                let msg = fmt_stats(
                    &mut buf,
                    sink.dropped,
                    capture.peek_dropped_total(),
                );
                wire::encode_log(msg, &mut sink);
                last_stats = embassy_time::Instant::now();
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

