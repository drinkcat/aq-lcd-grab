#![no_std]
#![no_main]

mod capture;

use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{Config as UsartConfig, Uart};
use embassy_stm32::Config;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use panic_halt as _;

use capture::{Capture, CapturePins, RING_CAP};
use wire::{Encoder, HOST_CMD_START, HOST_CMD_STATS, HOST_CMD_STOP, Sink};

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

/// TX ring size. Power of two for cheap wrap masking.
const TX_RING: usize = 2048;
const TX_RING_MASK: usize = TX_RING - 1;

/// USART1_TX DMA: DMA1 Channel 4. The metapac `ch()` index is 0-based,
/// so CH4 is `ch(3)` (matches capture.rs using `ch(4)` for CH5).
fn tx_dma_ch() -> embassy_stm32::pac::bdma::Ch {
    embassy_stm32::pac::DMA1.ch(3)
}

/// `Sink` backed by a software circular buffer that a re-armed,
/// non-circular DMA transfer drains to USART1->DR.
///
/// `push` writes one byte at `tail` (wrapping). If the ring is full it
/// spins until the in-flight DMA has drained a slot — the DMA advances
/// in hardware, independently of the executor, so the spin always makes
/// progress and never deadlocks. After writing, if the DMA channel is
/// idle (previous transfer finished, EN cleared by hardware at NDTR=0),
/// it arms a fresh transfer over the contiguous pending span. The DMA
/// stops itself at the span's end, so the wire goes silent when there's
/// nothing to send — no stale bytes (the bug a free-running circular
/// DMA caused).
struct UartSink {
    buf: &'static mut [u8; TX_RING],
    /// Start of the unsent region = start of the in-flight transfer (or
    /// of the next one to arm). Advanced past a transfer once it ends.
    head: usize,
    /// Next free slot for `push`.
    tail: usize,
    /// Length of the in-flight transfer; 0 when the channel is idle.
    inflight: usize,
}

impl UartSink {
    fn new(buf: &'static mut [u8; TX_RING]) -> Self {
        Self { buf, head: 0, tail: 0, inflight: 0 }
    }

    /// DMA channel index (0-based) for the ISR/IFCR flag bits. CH4.
    const DMA_IDX: usize = 3;

    /// True while a transfer is in flight and not yet complete. On F1
    /// the EN bit does **not** auto-clear at transfer-complete, so we
    /// can't use it as the idle signal — we track our own `inflight`
    /// and the hardware TC flag instead.
    fn inflight_busy(&self) -> bool {
        use embassy_stm32::pac;
        self.inflight > 0 && !pac::DMA1.isr().read().tcif(Self::DMA_IDX)
    }

    /// If the channel is idle, finalise the just-finished transfer (if
    /// any) and arm the next contiguous span of pending bytes.
    fn kick_dma(&mut self) {
        use embassy_stm32::pac;
        if self.inflight_busy() {
            return; // transfer still running
        }

        // A transfer finished (or none was running). Disable the channel
        // (EN doesn't auto-clear on F1), clear its TC flag, and advance
        // head past the bytes that just drained.
        if self.inflight > 0 {
            tx_dma_ch().cr().modify(|w| w.set_en(false));
            pac::DMA1.ifcr().write(|w| w.set_tcif(Self::DMA_IDX, true));
            self.head = (self.head + self.inflight) & TX_RING_MASK;
            self.inflight = 0;
        }

        if self.head == self.tail {
            return; // nothing pending — leave the channel idle (silent)
        }
        // Contiguous span from head up to either tail or the buffer end
        // (a wrap is sent as a second transfer on the next kick).
        let end = if self.tail > self.head { self.tail } else { TX_RING };
        let len = end - self.head;

        let ch = tx_dma_ch();
        ch.mar().write_value(self.buf.as_ptr() as u32 + self.head as u32);
        ch.ndtr().write(|w| w.set_ndt(len as u16));
        ch.cr().modify(|w| w.set_en(true));
        self.inflight = len;
    }

    /// Free slots in the ring, accounting for how far the in-flight DMA
    /// has drained (read live from NDTR).
    fn free(&self) -> usize {
        // Bytes of the in-flight transfer still pending in the ring.
        let remaining = if self.inflight > 0 {
            tx_dma_ch().ndtr().read().ndt() as usize
        } else {
            0
        };
        // Live head = committed head advanced by what the DMA drained.
        let live_head = (self.head + (self.inflight - remaining)) & TX_RING_MASK;
        // One slot is always kept free to disambiguate full vs empty.
        (live_head + TX_RING - self.tail - 1) & TX_RING_MASK
    }
}

impl Sink for UartSink {
    fn push(&mut self, b: u8) -> bool {
        // Spin until a slot frees up. The in-flight DMA drains the ring
        // in hardware while we wait, so this always makes progress.
        while self.free() == 0 {
            // Re-arm if the transfer finished while we spun (so the next
            // span starts draining and frees more slots).
            self.kick_dma();
        }
        self.buf[self.tail] = b;
        self.tail = (self.tail + 1) & TX_RING_MASK;
        self.kick_dma();
        true
    }
    // `flush` is a no-op: the DMA drains the ring on its own.
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

    // USART1 @ 921600. The encoder pushes bytes into a software ring
    // (UartSink) which a re-armed, non-circular DMA on DMA1_CH4
    // (USART1_TX) drains to USART1->DR. Each transfer ships exactly the
    // pending bytes and then stops, so the wire goes silent when idle —
    // a free-running circular DMA would retransmit stale bytes. This
    // keeps the per-byte TXE ISR off the capture hot path. CH4 is free:
    // capture owns CH5 (TIM2_CH1) and CH7 (TIM2_CH2).
    //
    // RX (host START/STOP) is not wired here: USART1_RX's DMA is CH5,
    // which capture needs, so RX must stay interrupt-driven and is
    // currently disabled along with the rx_fut PERF-test scaffold below.
    //
    // `UartTx::new_blocking` configures USART1 (baud, PA9 AF, UE+TE); we
    // drive the DMA channel by hand rather than via its blocking write.
    // `Uart::new_blocking` configures USART1 (baud, UE+TE+RE, PA9/PA10
    // pin AFs) and splits into TX + RX halves. We keep `_tx` only for
    // its USART config (TX is driven by the DMA below, not its blocking
    // write); `rx` is polled non-blocking in the cap loop for the rare
    // single-byte host commands (interrupt RX would need DMA1_CH5, which
    // capture owns).
    let mut usart_cfg = UsartConfig::default();
    usart_cfg.baudrate = 921600;
    let usart = Uart::new_blocking(p.USART1, p.PA10, p.PA9, usart_cfg).unwrap();
    let (_tx, rx) = usart.split();
    // `rx` must stay alive: dropping a UartRx disconnects the pin and
    // decrements the USART's refcount (disabling it). We read received
    // bytes via the PAC (RXNE/DR) in the cap loop rather than through
    // `rx`'s API, so it's otherwise unused.
    let _rx = rx;

    // One-time DMA channel + USART setup. `p.DMA1_CH4` is consumed to
    // mark the channel owned; we then drive its registers directly.
    let _tx_dma = p.DMA1_CH4;
    {
        use embassy_stm32::pac;
        // USART requests a DMA transfer on TXE.
        pac::USART1.cr3().modify(|w| w.set_dmat(true));
        let ch = tx_dma_ch();
        // Peripheral = USART1 data register (fixed); memory addr + count
        // are set per transfer in UartSink::kick_dma.
        ch.par().write_value(pac::USART1.dr().as_ptr() as u32);
        ch.cr().write(|w| {
            w.set_dir(pac::bdma::vals::Dir::FROM_MEMORY); // mem -> peripheral
            w.set_minc(true);   // walk the ring buffer
            w.set_pinc(false);  // fixed DR
            w.set_circ(false);  // one-shot: stop at NDTR=0
            w.set_pl(pac::bdma::vals::Pl::LOW); // never preempt capture
            // 8-bit on both ends (defaults to byte; set explicitly).
            w.set_msize(pac::bdma::vals::Size::BITS8);
            w.set_psize(pac::bdma::vals::Size::BITS8);
        });
    }

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

    // TX DMA ring storage. The DMA reads directly out of this buffer.
    static mut TX_RING_BUF: [u8; TX_RING] = [0; TX_RING];
    let tx_ring_buf = unsafe { &mut *core::ptr::addr_of_mut!(TX_RING_BUF) };

    let mut encoder = Encoder::default();
    let mut sink = UartSink::new(tx_ring_buf);
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

    // Boot STOPPED: the host drives START/STOP over the control path
    // (polled from RXNE in the cap loop below).
    let mut state = StreamState::Stopped;
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
            // Poll the USART receiver (RXNE) for host command bytes.
            // Interrupt RX would need DMA1_CH5 / an ISR; capture owns
            // CH5 and commands are rare + latency-tolerant, so a poll
            // per cap-loop iteration is plenty. Reading DR clears RXNE.
            {
                use embassy_stm32::pac;
                while pac::USART1.sr().read().rxne() {
                    let b = pac::USART1.dr().read().0 as u8;
                    let cmd = match b {
                        HOST_CMD_START => Some(HostCmd::Start),
                        HOST_CMD_STOP => Some(HostCmd::Stop),
                        HOST_CMD_STATS => Some(HostCmd::Stats),
                        _ => None, // unknown byte, ignore
                    };
                    if let Some(cmd) = cmd {
                        let _ = CMD_QUEUE.try_send(cmd);
                    }
                }
            }

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

            // In STOPPED state, nothing to capture. Clear accumulated
            // drops, kick the TX DMA so any staged acks (STARTED/STOPPED)
            // ship, and sleep a tick so we don't spin.
            if state != StreamState::Streaming {
                sink.kick_dma();
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

            // Kick the TX DMA so the tail of this iteration's output
            // (and any wrap remainder) ships even if no further `push`
            // arrives — otherwise a trailing frame would sit unsent on an
            // idle bus until the next byte is produced.
            sink.kick_dma();

            // Yield once per iteration so LED + RX get scheduled.
            // `read_available` only awaits when both rings are empty;
            // when data is plentiful it returns immediately. Without
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

