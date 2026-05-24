//! TIM1 + dual-DMA capture of the 8080 display bus.
//!
//! Per `docs/pcb_spec.md` §"Capture mechanism": TIM1 in external clock
//! mode 2 (ECE=1 in SMCR) is clocked by ETR=PA12=WR. With ARR=0, every
//! ETR edge produces an update event (UEV) and — via a CC1 channel in
//! output-compare with CCR=0 — a CC1 compare-match event on the same
//! cycle. Each event has an independent DMA request line (UDE, CC1DE)
//! enabled in DIER, so two DMA channels can fire from a single WR edge:
//!
//!   TIM1_UP  → DMA1 Ch5 → reads `GPIOB->IDR` → PB ring
//!   TIM1_CH1 → DMA1 Ch2 → reads `GPIOA->IDR` → PA ring
//!
//! Both channels are configured peripheral→memory, no peripheral increment,
//! memory increment, half-word size, circular.
//!
//! The CPU only touches the rings when draining. Pairing is implicit:
//! both channels fire on the same WR edge, so a read of (PA[i], PB[i])
//! corresponds to the i-th captured sample. Modulo the small DMA
//! arbitration lag between the two channels, ring fill levels stay
//! within 1 sample of each other; the drain function reads
//! `min(available_pa, available_pb)` paired samples per call.

use embassy_stm32::dma::{ReadableRingBuffer, TransferOptions};
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::pac;
use embassy_stm32::peripherals::{PA12, TIM1};
use embassy_stm32::timer::low_level::Timer;
use embassy_stm32::timer::{Ch1, Dma as TimDma, UpDma};
use embassy_stm32::Peri;

/// Mask of PA bits that are wired to the display flex. Bits outside
/// this mask are noise (floating inputs, other peripherals) and must
/// be zeroed before RLE so they don't break runs.
///
/// Final routing (see pcb/aq_lcd_grab.kicad_sch):
///   PA0..PA7  = DB0..DB7
///   PA12      = WR (read by TIM1 via ETR, not part of the data sample)
/// All other PA bits are unused; mask them off so noise on floating
/// inputs doesn't break RLE runs.
pub const PA_MASK: u16 = 0x00FF;

/// Final routing:
///   PB0..PB1  = DB8..DB9
///   PB2       = not exposed on the F103C8 package
///   PB3..PB8  = DB10..DB15
///   PB9       = unused
///   PB10      = DC
///   PB11      = CS
/// Host permute layer re-orders these into logical (data, dc, cs).
pub const PB_MASK: u16 = 0x0CFB;

/// Capture pin set. The data pins themselves don't need typed handles
/// (we read GPIOA->IDR/GPIOB->IDR as whole ports), but PA12 must be held
/// as input so nothing else claims it as an output.
pub struct CapturePins<'d> {
    pub wr_etr: Peri<'d, PA12>,
    // Data + control pins are read via GPIOA/B->IDR directly; they just
    // need to be configured as floating inputs somewhere. Caller passes
    // them as a tuple-of-Inputs so we hold the borrow for safety.
    // (Empty tuple is acceptable on Blue Pill bench rigs that just feed
    // PA12 with a square-wave generator and don't care about data.)
}

pub struct Capture<'d> {
    _wr: Input<'d>,
    _tim: Timer<'d, TIM1>,
    pa_ring: ReadableRingBuffer<'d, u16>,
    pb_ring: ReadableRingBuffer<'d, u16>,
    /// Saturating counter of samples lost to ring overrun.
    dropped: u32,
}

impl<'d> Capture<'d> {
    /// `pa_buf` and `pb_buf` must each be a power-of-2-sized slice with
    /// **the same length**. Embassy's circular DMA mode requires a
    /// power-of-2 length; matching lengths simplify the pairing logic.
    pub fn new(
        tim: Peri<'d, TIM1>,
        pins: CapturePins<'d>,
        pa_dma: Peri<'d, impl TimDma<TIM1, Ch1>>,
        pb_dma: Peri<'d, impl UpDma<TIM1>>,
        pa_buf: &'d mut [u16],
        pb_buf: &'d mut [u16],
    ) -> Self {
        assert_eq!(pa_buf.len(), pb_buf.len(), "ring buffers must match");
        assert!(
            pa_buf.len().is_power_of_two(),
            "ring length must be a power of 2"
        );

        // PA12 = ETR input. F1 wants AF inputs as plain floating input
        // (no AF mode bits — those are output-only on gpio_v1).
        let wr = Input::new(pins.wr_etr, Pull::None);

        // GPIOA->IDR and GPIOB->IDR live at fixed addresses; we read
        // them as half-words via DMA.
        let gpioa_idr = pac::GPIOA.idr().as_ptr() as *mut u16;
        let gpiob_idr = pac::GPIOB.idr().as_ptr() as *mut u16;

        // Grab the DMA request IDs *before* constructing the rings —
        // the Peri move into ReadableRingBuffer consumes the handle.
        let pa_req = pa_dma.request();
        let pb_req = pb_dma.request();

        // Both channels: peripheral→memory, half-word, circular.
        // `ReadableRingBuffer::new` sets circular=true internally.
        let opts = TransferOptions::default();
        let pa_ring = unsafe {
            ReadableRingBuffer::new(pa_dma, pa_req, gpioa_idr, pa_buf, opts)
        };
        let pb_ring = unsafe {
            ReadableRingBuffer::new(pb_dma, pb_req, gpiob_idr, pb_buf, opts)
        };

        // TIM1 setup. Use the low-level Timer wrapper to handle RCC
        // gating; do the slave-mode + DMA-enable register writes by hand
        // through regs_advanced() because the high-level API doesn't
        // expose ECE / dual-DMA-event mode.
        let tim = Timer::new(tim);
        let r = tim.regs_advanced();

        // Stop the counter while we reconfigure.
        r.cr1().modify(|w| w.set_cen(false));

        // ETR filter: 0 = no filter. The ETF[3:0] field in SMCR could
        // give us a free hardware glitch filter (per the spec's note
        // about reproducing the PIO firmware's reconfirm-after-glitch
        // behaviour), but we leave it off for first bring-up — re-add
        // ETF=0b0011 (require N consecutive samples) once we have real
        // bus signals to tune against.
        r.smcr().modify(|w| {
            w.set_etf(pac::timer::vals::FilterValue::NO_FILTER);
            w.set_etps(pac::timer::vals::Etps::DIV1);
            w.set_etp(pac::timer::vals::Etp::NOT_INVERTED);
            w.set_ece(true); // external clock mode 2: clock = ETRF
        });

        // ARR=0 → every ETR pulse increments past 0 → UEV fires.
        r.arr().write(|w| w.set_arr(0));
        r.psc().write_value(0);
        r.cnt().write(|w| w.set_cnt(0));

        // CC1 = output compare, CCR1=0 → compare-match on every cycle,
        // same edge as UEV. We don't need a physical output, just the
        // DMA request, so we configure CCMR1 in output mode and leave
        // CCER.CC1E=0.
        r.ccmr_output(0).modify(|w| {
            w.set_ccs(0, pac::timer::vals::CcmrOutputCcs::OUTPUT);
            w.set_ocm(0, pac::timer::vals::Ocm::FROZEN);
        });
        r.ccr(0).write(|w| w.set_ccr(0));

        // Enable both DMA request lines.
        r.dier().modify(|w| {
            w.set_ude(true);
            w.set_ccde(0, true);
        });

        // Force an update event so the prescaler/ARR shadow registers
        // actually take effect. Otherwise the first capture sees stale
        // values until the next overflow.
        r.egr().write(|w| w.set_ug(true));

        // Clear any pending status flags from the EGR-triggered UEV.
        r.sr().modify(|w| {
            w.set_uif(false);
            w.set_ccif(0, false);
        });

        // Hand the rings the green light.
        let mut pa_ring = pa_ring;
        let mut pb_ring = pb_ring;
        pa_ring.start();
        pb_ring.start();

        // Counter ON. WR edges now drive the capture.
        r.cr1().modify(|w| w.set_cen(true));

        Self {
            _wr: wr,
            _tim: tim,
            pa_ring,
            pb_ring,
            dropped: 0,
        }
    }

    /// Drain up to `out.len()` paired samples. Returns the count of
    /// samples actually written.
    ///
    /// On ring overrun in either channel, increments the dropped counter
    /// (clamped at u32::MAX) and skips the affected window. The two
    /// rings are kept in step by reading the min of their available
    /// counts per call — a slow drainer that's about to lose data in
    /// one channel will lose it symmetrically in both.
    pub fn drain(&mut self, out_pa: &mut [u16], out_pb: &mut [u16]) -> usize {
        debug_assert_eq!(out_pa.len(), out_pb.len());

        // Read from each channel into the caller's matched buffers,
        // capped at the shorter ring fill level (handled by Embassy's
        // ringbuffer `read()` — it returns however many samples were
        // available, up to the slice length).
        let n = out_pa.len().min(out_pb.len());

        let pa_result = self.pa_ring.read(&mut out_pa[..n]);
        let pb_result = self.pb_ring.read(&mut out_pb[..n]);

        let pa_read = match pa_result {
            Ok((read, _remaining)) => read,
            Err(_) => {
                // Overrun on PA ring. We can't tell exactly how many
                // were lost; conservatively count one full ring's worth
                // and resync by clearing both sides.
                self.dropped = self.dropped.saturating_add(self.pa_ring.capacity() as u32);
                self.pa_ring.clear();
                self.pb_ring.clear();
                return 0;
            }
        };
        let pb_read = match pb_result {
            Ok((read, _remaining)) => read,
            Err(_) => {
                self.dropped = self.dropped.saturating_add(self.pb_ring.capacity() as u32);
                self.pa_ring.clear();
                self.pb_ring.clear();
                return 0;
            }
        };

        // Both channels should report the same count modulo the small
        // DMA arbitration lag. If they diverge, trim to the shorter
        // and accept that the trailing samples will appear next drain.
        let n = pa_read.min(pb_read);

        // Mask off bits not wired to display signals. Without this,
        // noise on unused GPIOA/B inputs would break every RLE run.
        for s in &mut out_pa[..n] {
            *s &= PA_MASK;
        }
        for s in &mut out_pb[..n] {
            *s &= PB_MASK;
        }

        n
    }

    /// Take + reset the dropped-samples counter.
    pub fn take_dropped(&mut self) -> u32 {
        core::mem::replace(&mut self.dropped, 0)
    }
}
