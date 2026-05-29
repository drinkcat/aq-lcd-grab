//! USART1 transport: DMA-driven TX sink + polled RX.
//!
//! TX bytes are staged in a software circular buffer that a re-armed,
//! non-circular DMA transfer (DMA1_CH4 = USART1_TX) drains to the USART
//! data register. Each transfer ships exactly the pending span and then
//! stops, so the wire goes silent when idle — a free-running circular
//! DMA would retransmit stale bytes. This keeps the per-byte TXE ISR off
//! the capture hot path.
//!
//! RX (host START/STOP/STATS commands) is polled from the RXNE flag:
//! interrupt RX would need DMA1_CH5, which the capture front-end owns,
//! and commands are rare + latency-tolerant.

use embassy_stm32::mode::Blocking;
use embassy_stm32::pac;
use embassy_stm32::peripherals::{PA9, PA10, USART1};
use embassy_stm32::usart::{Config as UsartConfig, Uart, UartRx, UartTx};
use embassy_stm32::Peri;

use wire::Sink;

/// TX ring size. Power of two for cheap wrap masking.
const TX_RING: usize = 2048;
const TX_RING_MASK: usize = TX_RING - 1;

/// DMA channel index (0-based) for ISR/IFCR flag bits and `ch()`. CH4 =
/// USART1_TX; the metapac index is 0-based (matches capture's ch(4) for
/// CH5).
const DMA_IDX: usize = 3;

fn tx_dma_ch() -> pac::bdma::Ch {
    pac::DMA1.ch(DMA_IDX)
}

/// `Sink` backed by a software circular buffer drained by a re-armed,
/// non-circular DMA transfer.
///
/// `push` writes one byte at `tail` (wrapping). If the ring is full it
/// spins until the in-flight DMA drains a slot — the DMA advances in
/// hardware, independently of the executor, so the spin always makes
/// progress and never deadlocks. When the channel is idle it arms a
/// fresh transfer over the contiguous pending span; the DMA stops at the
/// span's end, so the wire is silent when there's nothing to send.
///
/// Holds the split USART halves to keep them alive: dropping a `UartRx`
/// disconnects its pin and disables the USART; `UartTx` is only kept for
/// the config it set up (TX is driven by the DMA, not its blocking
/// write).
pub struct UartSink {
    buf: &'static mut [u8; TX_RING],
    /// Start of the unsent region = start of the in-flight transfer (or
    /// of the next one to arm). Advanced past a transfer once it ends.
    head: usize,
    /// Next free slot for `push`.
    tail: usize,
    /// Length of the in-flight transfer; 0 when the channel is idle.
    inflight: usize,
    _tx: UartTx<'static, Blocking>,
    _rx: UartRx<'static, Blocking>,
}

impl UartSink {
    /// Configure USART1 @ 921600 (TX on PA9, RX on PA10) and the TX DMA
    /// channel, returning a sink that drains its internal ring over DMA.
    pub fn new(
        usart: Peri<'static, USART1>,
        tx_pin: Peri<'static, PA9>,
        rx_pin: Peri<'static, PA10>,
        _tx_dma: Peri<'static, embassy_stm32::peripherals::DMA1_CH4>,
    ) -> Self {
        // TX DMA ring storage. The DMA reads directly out of this buffer.
        static mut TX_RING_BUF: [u8; TX_RING] = [0; TX_RING];
        let buf = unsafe { &mut *core::ptr::addr_of_mut!(TX_RING_BUF) };

        // `Uart::new_blocking` configures USART1 (baud, UE+TE+RE, pin
        // AFs) and splits into TX + RX halves. TX is driven by the DMA
        // below, not the blocking write; RX is polled via the PAC.
        let mut cfg = UsartConfig::default();
        cfg.baudrate = 921600;
        let usart = Uart::new_blocking(usart, rx_pin, tx_pin, cfg).unwrap();
        let (tx, rx) = usart.split();

        // One-time DMA channel + USART setup. `_tx_dma` marks the channel
        // owned; we drive its registers directly.
        pac::USART1.cr3().modify(|w| w.set_dmat(true)); // request on TXE
        let ch = tx_dma_ch();
        // Peripheral = USART1 data register (fixed); memory addr + count
        // are set per transfer in `kick_dma`.
        ch.par().write_value(pac::USART1.dr().as_ptr() as u32);
        ch.cr().write(|w| {
            w.set_dir(pac::bdma::vals::Dir::FROM_MEMORY); // mem -> peripheral
            w.set_minc(true); // walk the ring buffer
            w.set_pinc(false); // fixed DR
            w.set_circ(false); // one-shot: stop at NDTR=0
            w.set_pl(pac::bdma::vals::Pl::LOW); // never preempt capture
            w.set_msize(pac::bdma::vals::Size::BITS8);
            w.set_psize(pac::bdma::vals::Size::BITS8);
        });

        Self {
            buf,
            head: 0,
            tail: 0,
            inflight: 0,
            _tx: tx,
            _rx: rx,
        }
    }

    /// Poll the RXNE flag for one received byte. Returns `None` when the
    /// receiver is empty. Reading DR clears RXNE.
    pub fn poll_rx(&self) -> Option<u8> {
        if pac::USART1.sr().read().rxne() {
            Some(pac::USART1.dr().read().0 as u8)
        } else {
            None
        }
    }

    /// Arm the DMA if there's pending data and the channel is idle. Call
    /// at a quiescent point to flush the tail of a burst that `push`
    /// couldn't arm (because the previous transfer was still running).
    pub fn kick(&mut self) {
        self.kick_dma();
    }

    /// True while a transfer is in flight and not yet complete. On F1 the
    /// bdma EN bit does **not** auto-clear at transfer-complete, so we
    /// can't use it as the idle signal — we track `inflight` and the
    /// hardware TC flag instead.
    fn inflight_busy(&self) -> bool {
        self.inflight > 0 && !pac::DMA1.isr().read().tcif(DMA_IDX)
    }

    /// If the channel is idle, finalise the just-finished transfer (if
    /// any) and arm the next contiguous span of pending bytes.
    fn kick_dma(&mut self) {
        if self.inflight_busy() {
            return; // transfer still running
        }

        // A transfer finished (or none ran). Disable the channel (EN
        // doesn't auto-clear on F1), clear its TC flag, and advance head
        // past the bytes that just drained.
        if self.inflight > 0 {
            tx_dma_ch().cr().modify(|w| w.set_en(false));
            pac::DMA1.ifcr().write(|w| w.set_tcif(DMA_IDX, true));
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
            self.kick_dma();
        }
        self.buf[self.tail] = b;
        self.tail = (self.tail + 1) & TX_RING_MASK;
        self.kick_dma();
        true
    }
    // `flush` is a no-op: the DMA drains the ring on its own; the cap
    // loop calls `kick` to flush a trailing burst.
}
