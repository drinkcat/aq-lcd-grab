#![no_std]
#![no_main]

mod capture;

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

use capture::{Capture, CapturePins, RING_CAP};
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
/// ring. When the ring is full, `push` spins until the TX ISR drains a
/// byte (~10 µs at 921600 baud) — the ISR runs independently of the
/// executor, so a full ring never deadlocks. Never drops, so frames are
/// always intact.
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
                Ok(0) => continue,      // ring full — spin until ISR drains
                Ok(_) => return true,
                Err(_) => return false, // UART error; host will reset us
            }
        }
    }
    // `flush` defaults to a no-op: the TX ISR drains the ring on its own.
}

/// Capture ring lengths. 4096 half-words = 8 KiB per ring = 16 KiB
/// total. At 667 kHz peak, 4096 samples ≈ 6 ms of headroom for the
/// drain task to keep up. Can't go higher: F103C8 has 20 KiB SRAM
/// total and we still need UART_TX_BUF + stack + .bss.
/// `Capture::fast_drain` hardcodes the modulus mask to RING_CAP, so
/// these statics must match it.
static mut PA_BUF: [u16; RING_CAP] = [0; RING_CAP];
static mut PB_BUF: [u16; RING_CAP] = [0; RING_CAP];

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
    // Every wired data/control pin (see permute_f103 / README routing).
    // Type-erased to AnyPin; Capture::new configures each as a floating
    // input so undriven pins don't float and corrupt the IDR reads.
    let data_pins = [
        // GPIOA: PA1=DB8, PA2..4=DB11..13, PA5=CS
        p.PA1.into(),
        p.PA2.into(),
        p.PA3.into(),
        p.PA4.into(),
        p.PA5.into(),
        // GPIOB: PB0..1=DB14..15, PB5..12=DB0..7, PB13..14=DB9..10, PB15=DC
        p.PB0.into(),
        p.PB1.into(),
        p.PB5.into(),
        p.PB6.into(),
        p.PB7.into(),
        p.PB8.into(),
        p.PB9.into(),
        p.PB10.into(),
        p.PB11.into(),
        p.PB12.into(),
        p.PB13.into(),
        p.PB14.into(),
        p.PB15.into(),
    ];
    let mut capture = Capture::new(
        p.TIM2,
        CapturePins { wr_etr: p.PA0, data: data_pins },
        p.DMA1_CH5, // TIM2_CH1 (input capture on TI1) -> PA ring
        p.DMA1_CH7, // TIM2_CH2 (input capture on TI1, alt) -> PB ring
        pa_buf,
        pb_buf,
    );

    let mut encoder = Encoder::default();
    let mut sink = UartSink::new(&mut tx);
    encoder.log("aq-lcd-grab stm32 firmware booted, awaiting START", &mut sink);

    // Disabled: trying to see if cap_fut alone gets higher
    // throughput without contention from the executor having to wake
    // these futures on their timers/IRQs.
    //
    // // LED task — visual liveness, also signals state.
    // let led_fut = async {
    //     loop {
    //         let interval = if STREAMING.load(core::sync::atomic::Ordering::Relaxed) {
    //             100
    //         } else {
    //             500
    //         };
    //         led.toggle();
    //         Timer::after_millis(interval).await;
    //     }
    // };
    //
    // // UART RX task — single-byte commands from the host.
    // let rx_fut = async {
    //     let mut byte = [0u8; 1];
    //     loop {
    //         if <_ as Read>::read(&mut rx, &mut byte).await.is_err() {
    //             continue;
    //         }
    //         let cmd = match byte[0] {
    //             HOST_CMD_START => HostCmd::Start,
    //             HOST_CMD_STOP => HostCmd::Stop,
    //             HOST_CMD_STATS => HostCmd::Stats,
    //             _ => continue, // unknown command, ignore
    //         };
    //         CMD_QUEUE.send(cmd).await;
    //     }
    // };
    let _ = &led;
    let _ = &rx;

    // PERF test: force-start streaming since rx_fut is disabled and
    // no START command will ever arrive. Host parser still needs the
    // STARTED ack to sync.
    let mut state = StreamState::Streaming;
    STREAMING.store(true, core::sync::atomic::Ordering::Relaxed);
    encoder.started(&mut sink);
    let cap_fut = async {
        let mut samples = [0u32; DRAIN_CHUNK];
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
        // DMA transfer-error event counters. take_dma_teif clears
        // the hardware flags each call, so these are cumulative
        // "events seen" counts since boot. Non-zero means a DMA
        // request was lost mid-burst (AHB error) → samples missing.
        let mut pa_teif_count: u32 = 0;
        let mut pb_teif_count: u32 = 0;
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
                        encoder.started(&mut sink);
                    }
                    HostCmd::Stop => {
                        if state == StreamState::Streaming {
                            encoder.flush(&mut sink);
                            state = StreamState::Stopped;
                            STREAMING.store(false, core::sync::atomic::Ordering::Relaxed);
                        }
                        encoder.stopped(&mut sink);
                    }
                    HostCmd::Stats => {
                        let mut buf = [0u8; 96];
                        let msg = fmt_stats(
                            &mut buf,
                            capture.peek_dropped_total(),
                            pa_teif_count,
                            pb_teif_count,
                        );
                        encoder.log(msg, &mut sink);
                    }
                }
            }

            // Wait until DMA has at least one paired sample (or a
            // 10 ms housekeeping timeout). `read_available` wakes on
            // the DMA half-/full-transfer IRQ and drains whatever's
            // there into packed u32 samples — no fixed batch wait.
            // While we wait, the executor parks us and runs LED/RX.
            //
            // On timeout, still do a sync `fast_drain` — samples may
            // have landed in the rings but not yet crossed the
            // N/2 wake mark. The IRQ won't fire for them; only a
            // direct ring read can pick them up.
            let mut n = match embassy_futures::select::select(
                capture.read_available(&mut samples),
                embassy_time::Timer::after_millis(10),
            )
            .await
            {
                embassy_futures::select::Either::First(n) => n,
                embassy_futures::select::Either::Second(_) => {
                    capture.fast_drain(&mut samples)
                }
            };

            // In STOPPED state, nothing to capture. Just clear any
            // accumulated drops and sleep a tick so we don't spin.
            if state != StreamState::Streaming {
                let _ = capture.take_dropped();
                embassy_time::Timer::after_millis(1).await;
                continue;
            }

            if n == 0 {
                // Idle bus — flush any partial encoder state so the
                // host sees what we have, then run the heartbeat.
                encoder.flush(&mut sink);

                if last_idle_log.elapsed() >= embassy_time::Duration::from_secs(5) {
                    let cnt = capture.peek_cnt();
                    let (pa_ndtr, pb_ndtr) = capture.peek_dma_ndtr();
                    let mut buf = [0u8; 48];
                    let msg = fmt_idle3(&mut buf, cnt, pa_ndtr, pb_ndtr);
                    encoder.log(msg, &mut sink);
                }
            } else {
                // Tight backlog drain. As long as the previous drain
                // filled the chunk, the rings probably have more
                // sitting behind it — keep draining synchronously
                // until we get a partial chunk (rings ~empty). Avoids
                // going through the executor + IRQ wakeup path per
                // DRAIN_CHUNK samples when we're racing to catch up
                // on a burst.
                loop {
                    for &sample in &samples[..n] {
                        encoder.feed(sample, &mut sink);
                    }

                    if n < DRAIN_CHUNK {
                        break;
                    }
                    n = capture.fast_drain(&mut samples);
                    if n == 0 {
                        break;
                    }
                }

                let dropped = capture.take_dropped();
                if dropped > 0 {
                    encoder.overrun(dropped, &mut sink);
                }

                last_idle_log = embassy_time::Instant::now();
            }

            // Auto-STATS heartbeat every ~5 s while streaming.
            if last_stats.elapsed() >= embassy_time::Duration::from_secs(5) {
                let mut buf = [0u8; 96];
                let msg = fmt_stats(
                    &mut buf,
                    capture.peek_dropped_total(),
                    pa_teif_count,
                    pb_teif_count,
                );
                encoder.log(msg, &mut sink);
                last_stats = embassy_time::Instant::now();
            }

            // Sample (and clear) DMA TEIF — accumulate event counts
            // so a 5s heartbeat surfaces them via STATS.
            let (pa_te, pb_te) = capture.take_dma_teif();
            if pa_te { pa_teif_count = pa_teif_count.saturating_add(1); }
            if pb_te { pb_teif_count = pb_teif_count.saturating_add(1); }

            // Yield once per iteration so LED + RX get scheduled.
            // `read_available` only awaits when both rings are empty;
            // when data is plentiful it returns immediately, and
            // `Sink::push` busy-waits on a full UART ring. Without
            // this yield the cap loop can monopolise the executor.
            embassy_futures::yield_now().await;
        }
    };

    cap_fut.await;
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

/// Format `"stats: cap_dropped=X pa_teif=X pb_teif=X"` into `buf`
/// and return a &str. `buf` must be at least 64 bytes.
fn fmt_stats(buf: &mut [u8], cap_dropped: u32, pa_teif: u32, pb_teif: u32) -> &str {
    let mut pos = 0;
    let prefix = b"stats: ";
    buf[..prefix.len()].copy_from_slice(prefix);
    pos += prefix.len();
    pos += write_label_u32(&mut buf[pos..], b"cap_dropped=", cap_dropped);
    buf[pos] = b' '; pos += 1;
    pos += write_label_u32(&mut buf[pos..], b"pa_teif=", pa_teif);
    buf[pos] = b' '; pos += 1;
    pos += write_label_u32(&mut buf[pos..], b"pb_teif=", pb_teif);
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

