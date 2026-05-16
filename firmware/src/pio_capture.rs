//! PIO program for capturing the target device's 16-bit 8080 MCU bus.
//!
//! Pin assignment (consecutive GPIOs required for `in pins, N`):
//!
//!     GPIO 0..=15 -> DB0..=DB15  (16-bit data bus)
//!     GPIO 16     -> D/C (RS)     [physically wired to CS — see decoder.rs]
//!     GPIO 17     -> CS           [physically wired to DC]
//!     GPIO 18     -> WR (write strobe — sample trigger)
//!
//! Each captured word in the RX FIFO is laid out (LSB first):
//!
//!     bit  17 16 15 ............... 0
//!          CS DC DB15 ............ DB0
//!
//! Upper 14 bits of the 32-bit word are zero (autopush threshold = 18).
//!
//! Sampling on WR rising edge — 8080 spec says data is valid then.

use embassy_rp::Peri;
use embassy_rp::dma::{self, Channel};
use embassy_rp::pac::dma::vals;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{Common, Config, Pio, ShiftConfig, ShiftDirection, StateMachine};

/// One sample as it lands in the FIFO.
pub type Sample = u32;

/// PIO + DMA running in ring mode: DMA writes continuously into a
/// power-of-2-aligned buffer, wrapping on its own. The CPU polls the
/// DMA's `write_addr` to find new samples.
pub struct RingCapture<'d> {
    _common: Common<'d, PIO0>,
    _sm: StateMachine<'d, PIO0, 0>,
    _dma: Channel<'d>,
    /// Cached DMA channel register block.
    regs: embassy_rp::pac::dma::Channel,
    /// Pointer to the start of the ring buffer (DMA write base).
    base: *mut Sample,
    /// log2(buffer length in samples). `len = 1 << log2_len`.
    log2_len: u8,
    /// Next sample index we will read out.
    read_pos: u32,
    /// Monotonic count of samples we know the writer dropped on us.
    /// Saturates at u32::MAX (which would be >4 billion samples, so
    /// practically unbounded).
    dropped_samples: u32,
}

impl<'d> RingCapture<'d> {
    /// `buf` must be a power-of-2-sized slice, aligned to (len * 4) bytes.
    /// RP2350 ring_size is in bytes — len_in_bytes must be a power of 2.
    pub fn new<DmaCh>(
        pio: Pio<'d, PIO0>,
        dma_peri: Peri<'d, DmaCh>,
        pins: CapturePins<'d>,
        dma_irqs: impl embassy_rp::interrupt::typelevel::Binding<
            <DmaCh as dma::ChannelInstance>::Interrupt,
            dma::InterruptHandler<DmaCh>,
        > + 'd,
        buf: &'static mut [Sample],
    ) -> Self
    where
        DmaCh: dma::ChannelInstance,
    {
        // Validate: power-of-2 length and aligned address.
        let len = buf.len();
        assert!(
            len.is_power_of_two(),
            "ring buffer length must be a power of 2"
        );
        let len_bytes = core::mem::size_of_val(buf);
        let base_addr = buf.as_mut_ptr() as usize;
        assert!(
            base_addr.is_multiple_of(len_bytes),
            "ring buffer must be aligned to its size in bytes (len*4)"
        );
        let log2_len = len.trailing_zeros() as u8;
        let log2_len_bytes = len_bytes.trailing_zeros() as u8;
        assert!(log2_len_bytes <= 31, "ring buffer too large");

        let Pio {
            mut common,
            mut sm0,
            ..
        } = pio;

        let prg = pio_asm!(
            ".wrap_target",
            "    wait 0 gpio 18",
            "    wait 1 gpio 18",
            "    in pins, 18",
            ".wrap",
        );

        let loaded = common.load_program(&prg.program);

        let p_in: [_; 18] = [
            common.make_pio_pin(pins.db0),
            common.make_pio_pin(pins.db1),
            common.make_pio_pin(pins.db2),
            common.make_pio_pin(pins.db3),
            common.make_pio_pin(pins.db4),
            common.make_pio_pin(pins.db5),
            common.make_pio_pin(pins.db6),
            common.make_pio_pin(pins.db7),
            common.make_pio_pin(pins.db8),
            common.make_pio_pin(pins.db9),
            common.make_pio_pin(pins.db10),
            common.make_pio_pin(pins.db11),
            common.make_pio_pin(pins.db12),
            common.make_pio_pin(pins.db13),
            common.make_pio_pin(pins.db14),
            common.make_pio_pin(pins.db15),
            common.make_pio_pin(pins.dc),
            common.make_pio_pin(pins.cs),
        ];
        let _wr = common.make_pio_pin(pins.wr);
        let p_refs: [&_; 18] = core::array::from_fn(|i| &p_in[i]);

        let mut cfg = Config::default();
        cfg.use_program(&loaded, &[]);
        cfg.set_in_pins(&p_refs);
        cfg.shift_in = ShiftConfig {
            auto_fill: true,
            threshold: 18,
            direction: ShiftDirection::Left,
        };

        sm0.set_config(&cfg);
        sm0.set_enable(true);

        // Channel number from the type's compile-time const (the
        // peripheral has not been claimed yet via `Channel::new`, so
        // call the trait function on the type).
        let ch_num = <DmaCh as dma::ChannelInstance>::number();

        // Construct the DMA channel without using embassy-rp's high-level
        // Transfer API — we want a free-running DMA with ring-write.
        let dma = Channel::new(dma_peri, dma_irqs);
        let regs = dma.regs();

        let dreq = sm0.rx_treq();
        let pio_rxf = sm0.rx_fifo_ptr() as u32;

        regs.read_addr().write_value(pio_rxf);
        regs.write_addr().write_value(base_addr as u32);
        regs.trans_count().write(|w| {
            // TRIGGER_SELF: when count hits 0 the channel re-triggers
            // itself. Combined with the ring_size wrap on the write
            // address, this gives a free-running ring buffer.
            w.set_mode(vals::TransCountMode::TRIGGER_SELF);
            w.set_count(len as u32);
        });
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        regs.ctrl_trig().write(|w| {
            w.set_treq_sel(dreq);
            w.set_data_size(vals::DataSize::SIZE_WORD);
            w.set_incr_read(false);
            w.set_incr_write(true);
            // ring_sel=true wraps the WRITE address. ring_sel=false
            // would wrap the (peripheral) read pointer instead — which
            // is not ring-aligned, AHB-errors on the first transfer,
            // and silently bricks the firmware via the DMA IRQ panic.
            w.set_ring_sel(true);
            w.set_ring_size(log2_len_bytes);
            // chain_to == self disables external chaining.
            w.set_chain_to(ch_num);
            // Re-trigger fires the per-channel completion IRQ; we
            // don't care, so silence it.
            w.set_irq_quiet(true);
            w.set_bswap(false);
            w.set_en(true);
        });
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

        Self {
            _common: common,
            _sm: sm0,
            _dma: dma,
            regs,
            base: buf.as_mut_ptr(),
            log2_len,
            read_pos: 0,
            dropped_samples: 0,
        }
    }

    fn len_mask(&self) -> u32 {
        (1u32 << self.log2_len) - 1
    }

    /// Returns the DMA's current write index (sample-position relative
    /// to the ring base, modulo `len`).
    pub fn write_pos(&self) -> u32 {
        let waddr = self.regs.write_addr().read();
        let byte_off = waddr.wrapping_sub(self.base as u32);
        (byte_off >> 2) & self.len_mask()
    }

    /// Number of samples available to read since the last call.
    pub fn available(&self) -> u32 {
        let w = self.write_pos();
        // Ring distance from read_pos to w.
        w.wrapping_sub(self.read_pos) & self.len_mask()
    }

    /// Pop up to `max` samples into `out`. Returns the count actually copied.
    ///
    /// Overrun handling: we can't directly tell from modular write/read
    /// positions whether the writer lapped us — after a full lap of
    /// `len` samples, `(w - r) & mask` is back to 0, indistinguishable
    /// from "no new data". So we use the buffer fill level as a
    /// high-water proxy: if at any drain `available()` exceeds 7/8 of
    /// the ring, we're at imminent risk of (or already past) overrun.
    /// In that case we count the unread tail as dropped, and resync
    /// `read_pos` to one slot past the writer — which is the oldest
    /// still-valid sample (about to be overwritten last, in `len - 1`
    /// more writes). This loses at most 1 sample of valid data but
    /// guarantees we never read across an overrun boundary.
    pub fn drain(&mut self, out: &mut [Sample]) -> usize {
        let mask = self.len_mask();
        let len = 1u32 << self.log2_len;
        let w = self.write_pos();
        let fill = w.wrapping_sub(self.read_pos) & mask;

        if fill >= (len / 8) * 7 {
            // Account everything we hadn't yet read as dropped (the
            // writer almost certainly clobbered an unknown portion of
            // it). Resync to the slot just past `w`, i.e. the oldest
            // valid sample — that slot will be overwritten last.
            self.dropped_samples = self.dropped_samples.saturating_add(fill);
            self.read_pos = w.wrapping_add(1) & mask;
        }

        let avail = self.available() as usize;
        let n = avail.min(out.len());
        for (i, slot) in out.iter_mut().take(n).enumerate() {
            let idx = (self.read_pos.wrapping_add(i as u32)) & mask;
            // SAFETY: idx < len, base is valid for `len` samples.
            *slot = unsafe { core::ptr::read_volatile(self.base.add(idx as usize)) };
        }
        self.read_pos = self.read_pos.wrapping_add(n as u32) & mask;
        n
    }

    /// Take the count of samples lost to ring overrun since the last
    /// call. Resets the internal counter.
    pub fn take_dropped(&mut self) -> u32 {
        core::mem::replace(&mut self.dropped_samples, 0)
    }
}

// SAFETY: the ring buffer is Send because `*mut Sample` is the only
// non-Send field. We only touch it from the capture task (single owner).
unsafe impl Send for RingCapture<'_> {}

pub struct CapturePins<'d> {
    pub db0: Peri<'d, embassy_rp::peripherals::PIN_0>,
    pub db1: Peri<'d, embassy_rp::peripherals::PIN_1>,
    pub db2: Peri<'d, embassy_rp::peripherals::PIN_2>,
    pub db3: Peri<'d, embassy_rp::peripherals::PIN_3>,
    pub db4: Peri<'d, embassy_rp::peripherals::PIN_4>,
    pub db5: Peri<'d, embassy_rp::peripherals::PIN_5>,
    pub db6: Peri<'d, embassy_rp::peripherals::PIN_6>,
    pub db7: Peri<'d, embassy_rp::peripherals::PIN_7>,
    pub db8: Peri<'d, embassy_rp::peripherals::PIN_8>,
    pub db9: Peri<'d, embassy_rp::peripherals::PIN_9>,
    pub db10: Peri<'d, embassy_rp::peripherals::PIN_10>,
    pub db11: Peri<'d, embassy_rp::peripherals::PIN_11>,
    pub db12: Peri<'d, embassy_rp::peripherals::PIN_12>,
    pub db13: Peri<'d, embassy_rp::peripherals::PIN_13>,
    pub db14: Peri<'d, embassy_rp::peripherals::PIN_14>,
    pub db15: Peri<'d, embassy_rp::peripherals::PIN_15>,
    pub dc: Peri<'d, embassy_rp::peripherals::PIN_16>,
    pub cs: Peri<'d, embassy_rp::peripherals::PIN_17>,
    pub wr: Peri<'d, embassy_rp::peripherals::PIN_18>,
}
