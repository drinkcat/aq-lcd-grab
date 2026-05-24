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
use wire::{Encoder, HOST_CMD_START, HOST_CMD_STOP, Sink};

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
}

/// `Sink` impl that pushes into the TX queue, dropping bytes if full.
struct QueueSink {
    dropped: u32,
}

impl Sink for QueueSink {
    fn push(&mut self, b: u8) -> bool {
        match TX_QUEUE.try_send(b) {
            Ok(()) => true,
            Err(_) => {
                self.dropped = self.dropped.saturating_add(1);
                false
            }
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
    // need Ch5 for TIM1_UP. Interrupt-driven RX is fine: host commands
    // are single-byte and rare. TX is also interrupt-driven; at 921600
    // baud the ~10 µs ISR cadence is well below the 64 MHz CPU's
    // critical-path budget.
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
        p.TIM1,
        CapturePins { wr_etr: p.PA12 },
        p.DMA1_CH2, // TIM1_CH1 -> PA ring
        p.DMA1_CH5, // TIM1_UP  -> PB ring
        pa_buf,
        pb_buf,
    );

    let mut encoder = Encoder::default();
    let mut sink = QueueSink { dropped: 0 };
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
                    // ~5 s heartbeat when idle; useful sign-of-life.
                    if state == StreamState::Streaming {
                        wire::encode_log("idle", &mut sink);
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
