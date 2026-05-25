#![no_std]
#![no_main]

mod capture;
mod wire;

use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{BufferedUart, BufferedUartTx, Config as UsartConfig};
use embassy_stm32::usart::BufferedInterruptHandler;
use embassy_stm32::{bind_interrupts, peripherals, Config};
use embedded_io::Write as _;
use embedded_io_async::Read;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use panic_halt as _;

use capture::{Capture, CapturePins};
use wire::{Encoder, HOST_CMD_START, HOST_CMD_STATS, HOST_CMD_STOP, Sink};

bind_interrupts!(struct Irqs {
    USART1 => BufferedInterruptHandler<peripherals::USART1>;
});

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

/// `Sink` that pushes each byte straight into the BufferedUart's TX
/// ring. `commit_frame` is a no-op: there's nothing to flush because
/// each byte was already enqueued by `push`. When the ring is full,
/// `push` spins on `blocking_write` until the ISR drains a byte
/// (~10 µs at 921600 baud).
struct UartSink<'a, 'd> {
    tx: &'a mut BufferedUartTx<'d>,
}

impl<'a, 'd> UartSink<'a, 'd> {
    fn new(tx: &'a mut BufferedUartTx<'d>) -> Self {
        Self { tx }
    }
}

impl<'a, 'd> Sink for UartSink<'a, 'd> {
    fn push(&mut self, b: u8) -> bool {
        loop {
            match self.tx.write(&[b]) {
                Ok(0) => continue, // ring full — spin until ISR drains
                Ok(_) => return true,
                Err(_) => return false, // UART error; host will reset us
            }
        }
    }
}

/// Capture ring lengths. 2048 half-words = 4 KiB per ring = 8 KiB
/// total. At 667 kHz peak, 2048 samples ≈ 3 ms of headroom — twice
/// the original 1.5 ms and enough to ride out the drain-thread
/// starvation we were hitting. Could go higher but F103C8 only has
/// 20 KiB SRAM and we also need UART_TX_BUF.
const RING_LEN: usize = 4096;
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
    // CPU's critical-path budget. The TX ring is the only outbound
    // buffer — the sink writes frames straight into it.
    static mut UART_TX_BUF: [u8; 2048] = [0; 2048];
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
    let mut sink = UartSink::new(&mut tx);
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
        // Time-based (not loop-tick-based) cadence for the periodic
        // diagnostic emits below — the cap loop's iteration rate
        // depends on DMA progress, so iteration counts aren't a
        // reliable clock.
        let mut last_idle_log = embassy_time::Instant::now();
        // Auto-emit a STATS log line every ~5 s while streaming, so
        // the host's run.log carries a trail of cap_dropped counters
        // even under sustained traffic (where the idle-tick heartbeat
        // below never fires).
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
                        let msg = fmt_stats(&mut buf, capture.peek_dropped_total());
                        wire::encode_log(msg, &mut sink);
                    }
                }
            }

            // In STOPPED state, nothing to capture. Just clear any
            // accumulated drops and sleep a tick so we don't spin.
            if state != StreamState::Streaming {
                let _ = capture.take_dropped();
                embassy_time::Timer::after_millis(10).await;
                continue;
            }

            // Wait until DMA has at least one paired sample (or a
            // 10 ms housekeeping timeout). `read_available` wakes on
            // the DMA half-/full-transfer IRQ and drains whatever's
            // there — no fixed batch wait. While we wait, the
            // executor parks us and runs LED/RX.
            let n = match embassy_futures::select::select(
                capture.read_available(&mut pa_buf, &mut pb_buf),
                embassy_time::Timer::after_millis(10),
            )
            .await
            {
                embassy_futures::select::Either::First(n) => n,
                embassy_futures::select::Either::Second(_) => 0,
            };

            if n == 0 {
                // Idle bus — flush any partial encoder state so the
                // host sees what we have, then run the heartbeat.
                encoder.flush(&mut sink);

                if last_idle_log.elapsed() >= embassy_time::Duration::from_secs(5) {
                    let cnt = capture.peek_cnt();
                    let (pa_ndtr, pb_ndtr) = capture.peek_dma_ndtr();
                    let mut buf = [0u8; 48];
                    let msg = fmt_idle3(&mut buf, cnt, pa_ndtr, pb_ndtr);
                    wire::encode_log(msg, &mut sink);
                }
            } else {
                for i in 0..n {
                    // pa = GPIOA->IDR low 16, pb = GPIOB->IDR low 16.
                    // Packing into one u32 matches the on-wire LE byte
                    // layout exactly; the host's permute layer
                    // unpacks them per-board.
                    let sample = pa_buf[i] as u32 | (pb_buf[i] as u32) << 16;
                    encoder.feed(sample, &mut sink);
                }

                let dropped = capture.take_dropped();
                if dropped > 0 {
                    wire::encode_overrun(dropped, &mut sink);
                }
                last_idle_log = embassy_time::Instant::now();
            }

            // Auto-STATS heartbeat every ~5 s while streaming.
            if last_stats.elapsed() >= embassy_time::Duration::from_secs(5) {
                let mut buf = [0u8; 64];
                let msg = fmt_stats(&mut buf, capture.peek_dropped_total());
                wire::encode_log(msg, &mut sink);
                last_stats = embassy_time::Instant::now();
            }

            // Yield once per iteration so LED + RX get scheduled.
            // `read_available` only awaits when both rings are empty;
            // when data is plentiful it returns immediately, and
            // `Sink::push` busy-waits on a full UART ring. Without
            // this yield the cap loop can monopolise the executor.
            embassy_futures::yield_now().await;
        }
    };

    join3(led_fut, rx_fut, cap_fut).await;
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

/// Format `"stats: cap_dropped=XXXXXXXX"` into `buf` and return a
/// &str. `buf` must be at least 28 bytes.
fn fmt_stats(buf: &mut [u8], cap_dropped: u32) -> &str {
    let mut pos = 0;
    let prefix = b"stats: ";
    buf[..prefix.len()].copy_from_slice(prefix);
    pos += prefix.len();
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

