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
use embassy_sync::signal::Signal;
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

/// Stream control: STOPPED → STREAMING and back. Signaled by RX task,
/// observed by capture task.
static STATE: Signal<ThreadModeRawMutex, StreamState> = Signal::new();

#[derive(Copy, Clone, PartialEq, Eq)]
enum StreamState {
    Stopped,
    Streaming,
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

    // Initial state.
    STATE.signal(StreamState::Stopped);
    let mut encoder = Encoder::default();
    let mut sink = QueueSink { dropped: 0 };
    wire::encode_log("aq-lcd-grab stm32 firmware booted, awaiting START", &mut sink);

    // LED task — visual liveness, also signals state.
    let led_fut = async {
        loop {
            let interval = match STATE.try_take().unwrap_or(StreamState::Stopped) {
                StreamState::Stopped => 500,
                StreamState::Streaming => 100,
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
            match byte[0] {
                HOST_CMD_START => STATE.signal(StreamState::Streaming),
                HOST_CMD_STOP => STATE.signal(StreamState::Stopped),
                _ => {} // unknown command, ignore
            }
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
            // Apply any pending state change.
            if let Some(new_state) = STATE.try_take() {
                if new_state != state {
                    match new_state {
                        StreamState::Streaming => {
                            encoder.reset();
                            wire::encode_started(&mut sink);
                        }
                        StreamState::Stopped => {
                            encoder.flush(&mut sink);
                            wire::encode_stopped(&mut sink);
                        }
                    }
                    state = new_state;
                    // Re-arm the signal for next observation.
                    STATE.signal(new_state);
                }
            }

            // Drain rings regardless of state — keeping them empty
            // prevents an overrun marker the moment we start.
            let n = capture.drain(&mut pa_buf, &mut pb_buf);

            if n > 0 && state == StreamState::Streaming {
                for i in 0..n {
                    encoder.feed(pa_buf[i], pb_buf[i], &mut sink);
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
