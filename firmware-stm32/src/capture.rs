//! TIM2 + dual-DMA capture of the 8080 display bus.
//!
//! Per `docs/pcb_spec.md` §"Capture mechanism": the capture timer runs
//! in external clock mode 2 (ECE=1 in SMCR), clocked by ETR=WR. With
//! ARR=0, every ETR edge produces an update event (UEV) and — via a
//! CC1 channel in output-compare with CCR=0 — a CC1 compare-match
//! event on the same cycle. Each event has an independent DMA request
//! line (UDE, CC1DE) enabled in DIER, so two DMA channels fire from a
//! single WR edge.
//!
//! WR is on **PA0 = TIM2_ETR** on both the Blue Pill bench rig and the
//! fab'd capture PCB.  (The Blue Pill ties PA12 to 3V3 through a 1.5 kΩ
//! USB-DP pull-up; using TIM2/PA0 on the PCB too keeps one firmware for
//! both boards.)
//!
//!   TIM2_UP  → DMA1 Ch2 → reads `GPIOB->IDR` → PB ring
//!   TIM2_CH1 → DMA1 Ch5 → reads `GPIOA->IDR` → PA ring
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

use core::future::poll_fn;
use core::task::Poll;
use embassy_stm32::Peri;
use embassy_stm32::dma::{Priority, ReadableRingBuffer, TransferOptions};
use embassy_stm32::gpio::{AnyPin, Input, Pull};
use embassy_stm32::pac;
use embassy_stm32::peripherals::{PA0, TIM2};
use embassy_stm32::timer::low_level::Timer;
use embassy_stm32::timer::{Ch1, Ch2, Dma as TimDma};

/// Mask of PA bits that are wired to the display flex. Bits outside
/// this mask are noise (floating inputs, other peripherals) and must
/// be zeroed before RLE so they don't break runs.
///
/// Blue Pill bench routing (see firmware-stm32/README.md):
///   PA0       = WR (TIM2 ETR; not part of the data sample)
///   PA1       = DB8 (G3)
///   PA2..PA4  = DB11..DB13 (R0..R2)
///   PA5       = CS (captured but unused in decode; masked out so its
///               toggling doesn't break RLE runs)
/// Wired data bits: PA1..PA4 (bits 1..4) = 0x001E. (PA1/PA2/PA3 are
/// TIM2_CH2/3/4 but the timer doesn't claim them — CC2 uses the
/// alternate TI1 mapping, CC3/CC4 are unused.)
pub const PA_MASK: u16 = 0x001E;

/// Blue Pill bench routing. GPIOB is the self-sufficient port — DC +
/// low byte DB0..DB7 + the top two red (R4/R3) and green (G5/G4) bits:
///   PB0..PB1   = DB14..DB15 (R3, R4)
///   PB2        = BOOT1; skipped
///   PB3, PB4   = JTAG TDO / NJTRST at reset; skipped so we don't have
///                to fiddle with AFIO SWJ_CFG
///   PB5..PB12  = DB0..DB7
///   PB13..PB14 = DB9..DB10 (G4, G5)
///   PB15       = DC (framing signal; kept in the mask)
pub const PB_MASK: u16 = 0xFFE3;

/// Number of data/control pins read via GPIOA/B->IDR (DB0..DB15 + DC +
/// CS = 18). PA0/WR is the ETR clock and held separately.
pub const DATA_PINS: usize = 18;

/// Capture pin set. We read GPIOA->IDR/GPIOB->IDR as whole ports, so
/// the data pins don't need *typed* handles — but they must be
/// configured as inputs and the config must persist, or an undriven
/// pin floats and IDR reads garbage. So the caller hands over every
/// wired data/control pin (type-erased to `AnyPin`); `Capture::new`
/// configures each as a floating input and holds it for `'d`. PA0/WR
/// is separate because it is also the TIM2 ETR clock.
pub struct CapturePins<'d> {
    pub wr_etr: Peri<'d, PA0>,
    pub data: [Peri<'d, AnyPin>; DATA_PINS],
}

pub struct Capture<'d> {
    _wr: Input<'d>,
    /// Data/control pins held as floating inputs. Never read through
    /// here (we read the whole port via IDR/DMA); held only so the
    /// input config persists for `'d` and the pins don't float.
    _data: [Input<'d>; DATA_PINS],
    _tim: Timer<'d, TIM2>,
    pa_ring: ReadableRingBuffer<'d, u16>,
    pb_ring: ReadableRingBuffer<'d, u16>,
    /// Raw pointers to the DMA buffers, captured before the slice was
    /// moved into the ring constructors. Used by `fast_drain` to read
    /// samples directly without going through embassy's per-sample
    /// `read_volatile` + `as_index` + `%cap` path. Length is
    /// `RING_CAP`, must be a power of two (asserted in `new`).
    pa_buf_ptr: *const u16,
    pb_buf_ptr: *const u16,
    /// Our own read indices (mod RING_CAP) for `fast_drain`. Kept
    /// in sync with the embassy ring's internal read_index — but
    /// since fast_drain bypasses the embassy ring entirely, embassy
    /// never sees the reads. We only use the embassy ring for the
    /// async waker (`read_available` → `set_waker`).
    pa_read_idx: usize,
    pb_read_idx: usize,
    /// Samples lost to ring overrun *since the last `take_dropped`*.
    /// Cap task drains this each tick to emit tag=0xFD overrun frames.
    dropped: u32,
    /// Lifetime cumulative count of dropped samples — never reset.
    /// Surfaced via STATS so the host can see whether ring overrun is
    /// the dominant loss mode.
    dropped_total: u32,
}

/// Hardcoded ring capacity. Must match the slice passed to
/// `Capture::new` and must be a power of two — `fast_drain` uses
/// `& (RING_CAP - 1)` for the modulus, so a non-pow2 cap would
/// silently corrupt indices.
pub const RING_CAP: usize = 4096;
const RING_MASK: usize = RING_CAP - 1;

impl<'d> Capture<'d> {
    /// `pa_buf` and `pb_buf` must each be a power-of-2-sized slice with
    /// **the same length**. Embassy's circular DMA mode requires a
    /// power-of-2 length; matching lengths simplify the pairing logic.
    pub fn new(
        tim: Peri<'d, TIM2>,
        pins: CapturePins<'d>,
        pa_dma: Peri<'d, impl TimDma<TIM2, Ch1>>,
        pb_dma: Peri<'d, impl TimDma<TIM2, Ch2>>,
        pa_buf: &'d mut [u16],
        pb_buf: &'d mut [u16],
    ) -> Self {
        assert_eq!(pa_buf.len(), pb_buf.len(), "ring buffers must match");
        assert_eq!(
            pa_buf.len(),
            RING_CAP,
            "ring length must equal RING_CAP — fast_drain hardcodes the modulus mask"
        );

        // PA0 = TIM2 ETR input. F1 wants AF inputs as plain floating
        // input (no AF mode bits — those are output-only on gpio_v1).
        let wr = Input::new(pins.wr_etr, Pull::None);

        // Configure every wired data/control pin as a floating input.
        // Without this they default to whatever reset/other-peripheral
        // state left them in; undriven pins float and corrupt the IDR
        // reads (seen as 0xffff floods that break RLE). The bus driver
        // is active, so Pull::None is correct — no pull to fight
        // the driver.
        let data = pins.data.map(|p| Input::new(p, Pull::None));

        // GPIOA->IDR and GPIOB->IDR live at fixed addresses; we read
        // them as half-words via DMA.
        let gpioa_idr = pac::GPIOA.idr().as_ptr() as *mut u16;
        let gpiob_idr = pac::GPIOB.idr().as_ptr() as *mut u16;

        // Snapshot the buffer pointers before the slices are moved
        // into ReadableRingBuffer (it consumes them). `fast_drain`
        // reads from these directly, bypassing embassy's per-sample
        // `read_volatile` + `as_index` + udiv-by-cap path.
        let pa_buf_ptr = pa_buf.as_ptr();
        let pb_buf_ptr = pb_buf.as_ptr();

        // Both channels: peripheral→memory, half-word, circular.
        // `ReadableRingBuffer::new` sets circular=true internally.
        //
        // DMA arbitration priority. On F1, equal software priority (PL)
        // breaks ties by channel number (lower = higher), so with the
        // default VeryHigh on both, PA (CH5) would beat PB (CH7). We want
        // the PB read serviced first on each WR edge, so give PB the
        // top PL and drop PA one notch below it. (The USART1_TX ring on
        // CH4 keeps the default Low, so it never preempts capture.)
        let mut pb_opts = TransferOptions::default();
        pb_opts.priority = Priority::VeryHigh;
        let mut pa_opts = TransferOptions::default();
        pa_opts.priority = Priority::High;
        let pa_ring = unsafe { ReadableRingBuffer::new(pa_dma, (), gpioa_idr, pa_buf, pa_opts) };
        let pb_ring = unsafe { ReadableRingBuffer::new(pb_dma, (), gpiob_idr, pb_buf, pb_opts) };

        // TIM2 setup. Use the low-level Timer wrapper to handle RCC
        // gating; do the slave-mode + DMA-enable register writes by hand
        // through regs_gp16() because the high-level API doesn't expose
        // ECE / dual-DMA-event mode. (TIM2 is a general-purpose 16-bit
        // timer, so we use regs_gp16 instead of TIM1's regs_advanced.)
        let tim = Timer::new(tim);
        let r = tim.regs_gp16();

        // Stop the counter while we reconfigure.
        r.cr1().modify(|w| w.set_cen(false));

        // ETR filter: FCK_INT_N8 (8 consecutive CK_INT samples). N2 made
        // corruption worse (2.6% -> 5.9%); trying a longer window to see
        // if it rejects whole glitch-bursts rather than nudging the
        // trigger into the next bus state. Ladder: N2/N4/N8 at CK_INT,
        // then FDTS_DIV* for much longer debounce.
        r.smcr().modify(|w| {
            w.set_etf(pac::timer::vals::FilterValue::FCK_INT_N8);
            w.set_etps(pac::timer::vals::Etps::DIV1);
            // Sample on WR *falling* edge, matching the Pico PIO
            // (`firmware/src/pio_capture.rs`). The bus driver
            // appears to deassert DC slightly before WR rises (8080
            // timing violation), so sampling at the rising edge sees
            // DC already back to "data" even for command bytes.
            // Sampling at the falling edge catches DC while it's
            // still asserted for the in-flight byte.
            w.set_etp(pac::timer::vals::Etp::INVERTED);
            w.set_ece(true); // external clock mode 2: clock = ETRF
        });

        // ARR isn't load-bearing here: we don't depend on UEV. Keep
        // it at 0xFFFF so the counter has room to wrap without
        // weirdness.
        r.arr().write(|w| w.set_arr(0xFFFF));
        r.psc().write_value(0);
        r.cnt().write(|w| w.set_cnt(0));

        // BOTH CC1 and CC2 are configured as input-capture on TI1
        // (the same pin as ETR — TIM2's CH1_ETR is multiplexed and
        // both ETR-clocking and input-capture read the same pin).
        // CC1 fires CC1DE → DMA1_CH5 → reads GPIOA->IDR.
        // CC2 (mapped to TI1 via the "alternate" CCS=TI3 setting)
        // fires CC2DE → DMA1_CH7 → reads GPIOB->IDR.
        // Both DMA transfers happen on the same WR edge so the rings
        // stay paired sample-for-sample.
        //
        // History: an earlier design used UEV→UDE→DMA1_CH2 for one
        // half and CC1→CC1DE for the other, with ARR=0 to make UEV
        // fire on every ETR edge. On F103 TIM2 (and probably others),
        // ARR=0 + ECE silently fails to fire UDE — measured 0 DMA
        // transfers in 30 s while CC1IF/CC1DE was happily firing
        // every edge. Switching to dual input-capture sidesteps the
        // issue entirely.
        r.ccmr_input(0).modify(|w| {
            // CC1 ← TI1 (normal). CCS=01.
            w.set_ccs(0, pac::timer::vals::CcmrInputCcs::TI4);
            w.set_icf(0, pac::timer::vals::FilterValue::NO_FILTER);
            w.set_icpsc(0, 0);
            // CC2 ← TI1 (alternate mapping, switches 2↔1). CCS=10.
            w.set_ccs(1, pac::timer::vals::CcmrInputCcs::TI3);
            w.set_icf(1, pac::timer::vals::FilterValue::NO_FILTER);
            w.set_icpsc(1, 0);
        });
        r.ccer().modify(|w| {
            // Falling edge for both, matching ETR (see SMCR.ETP comment).
            w.set_ccp(0, true);
            w.set_cce(0, true); // CC1 falling, enable
            w.set_ccp(1, true);
            w.set_cce(1, true); // CC2 falling, enable
        });

        // Enable both CC DMA request lines.
        r.dier().modify(|w| {
            w.set_ccde(0, true);
            w.set_ccde(1, true);
        });

        // Force an update event so the prescaler/ARR shadow registers
        // actually take effect. Otherwise the first capture sees stale
        // values until the next overflow.
        r.egr().write(|w| w.set_ug(true));

        // Clear any pending status flags from the EGR-triggered UEV.
        // Both CC1IF *and* CC2IF — missing the CC2 clear leaves a
        // stale interrupt flag pending, which the DMA controller
        // consumes as soon as we arm DMA1_CH7. That puts one ghost
        // sample at the head of the PB ring, shifting it by one
        // relative to PA for the rest of the session — every
        // captured (pa, pb) pair is mismatched.
        r.sr().modify(|w| {
            w.set_uif(false);
            w.set_ccif(0, false);
            w.set_ccif(1, false);
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
            _data: data,
            _tim: tim,
            pa_ring,
            pb_ring,
            pa_buf_ptr,
            pb_buf_ptr,
            pa_read_idx: 0,
            pb_read_idx: 0,
            dropped: 0,
            dropped_total: 0,
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
    #[allow(dead_code)] // embassy-ring drain path, kept as an alternative to fast_drain
    pub fn drain(&mut self, out_pa: &mut [u16], out_pb: &mut [u16]) -> usize {
        debug_assert_eq!(out_pa.len(), out_pb.len());

        // Read from each channel into the caller's matched buffers,
        // capped at the shorter ring fill level (handled by Embassy's
        // ringbuffer `read()` — it returns however many samples were
        // available, up to the slice length).
        let n = out_pa.len().min(out_pb.len());

        let pa_result = self.pa_ring.read(&mut out_pa[..n]);
        let pb_result = self.pb_ring.read(&mut out_pb[..n]);

        // On overrun in *either* ring, do a hard resync: stop TIM2
        // (so no more edges trigger DMA), clear both rings (which
        // re-anchors their read indices to the now-frozen DMA write
        // positions), restart TIM2. By gating the resync on the
        // *trigger source* rather than the DMA channels themselves,
        // we guarantee both rings get re-anchored to the same
        // edge-boundary — without this, the two soft-reset reads
        // sample each DMA's write position at non-atomic moments
        // and end up off by ±1 sample, causing every paired sample
        // from then on to come from two different WR edges.
        let overrun = pa_result.is_err() || pb_result.is_err();
        if overrun {
            self.hard_resync();
            return 0;
        }
        let pa_read = pa_result.map(|(r, _)| r).unwrap_or(0);
        let pb_read = pb_result.map(|(r, _)| r).unwrap_or(0);

        // Both channels should report the same count modulo the small
        // DMA arbitration lag. If they diverge, trim to the shorter
        // and accept that the trailing samples will appear next drain.
        // Caller masks each sample with PA_MASK / PB_MASK in the same
        // pass it packs PA|PB into a u32, so the unmasked bits don't
        // bounce through the stack twice.
        pa_read.min(pb_read)
    }

    /// Drain paired samples directly from the DMA buffers, packing
    /// into u32 (PA in low 16, PB in high 16, mask applied) — bypasses
    /// embassy's `read_volatile` + per-sample `%cap` + double `len()`
    /// path. Returns count written.
    ///
    /// Overrun detection: this reads NDTR on both channels and trusts
    /// that the gap between our read index and the DMA write position
    /// never exceeds `RING_CAP / 2`. We can't see TCIF (embassy's ISR
    /// consumes it), so we use a safety-margin heuristic instead of
    /// exact wrap-counting: if available samples > RING_CAP/2 we
    /// declare overrun. False positives possible if the caller falls
    /// behind by more than half a ring — but that *is* an overrun in
    /// the practical sense (drain rate < fill rate by 2× headroom),
    /// so triggering early is fine.
    pub fn fast_drain(&mut self, out: &mut [u32]) -> usize {
        let pa_ndtr = pac::DMA1.ch(4).ndtr().read().ndt() as usize;
        let pb_ndtr = pac::DMA1.ch(6).ndtr().read().ndt() as usize;

        // DMA was armed with RING_CAP transfers; NDTR counts down from
        // RING_CAP to 1, then auto-reloads. write_pos = next slot the
        // DMA will write to. NDTR==0 only momentarily; treat 0 as
        // RING_CAP-equivalent (i.e. write_pos = 0, just wrapped).
        let pa_write = (RING_CAP - pa_ndtr) & RING_MASK;
        let pb_write = (RING_CAP - pb_ndtr) & RING_MASK;

        let pa_avail = (pa_write.wrapping_sub(self.pa_read_idx)) & RING_MASK;
        let pb_avail = (pb_write.wrapping_sub(self.pb_read_idx)) & RING_MASK;

        // Overrun guard: if either channel's backlog exceeds half the
        // ring, assume the DMA is about to lap us (NDTR can't tell a
        // true lap from a large backlog) and hard-resync rather than
        // drain torn data. May false-positive on a legitimately large
        // burst, but that's the conservative choice — a resync shows up
        // in cap_dropped, whereas torn samples are silent corruption.
        if pa_avail > RING_CAP / 2 || pb_avail > RING_CAP / 2 {
            self.hard_resync();
            return 0;
        }

        let n = pa_avail.min(pb_avail).min(out.len());
        const SAMPLE_MASK: u32 = PA_MASK as u32 | ((PB_MASK as u32) << 16);

        let pa_base = self.pa_buf_ptr;
        let pb_base = self.pb_buf_ptr;
        let mut pa_i = self.pa_read_idx;
        let mut pb_i = self.pb_read_idx;
        for slot in &mut out[..n] {
            // SAFETY: pa_base/pb_base point to a [u16; RING_CAP] live
            // for 'd; pa_i/pb_i are always in 0..RING_CAP via &mask.
            let pa = unsafe { core::ptr::read_volatile(pa_base.add(pa_i)) };
            let pb = unsafe { core::ptr::read_volatile(pb_base.add(pb_i)) };
            *slot = ((pa as u32) | ((pb as u32) << 16)) & SAMPLE_MASK;
            pa_i = (pa_i + 1) & RING_MASK;
            pb_i = (pb_i + 1) & RING_MASK;
        }
        self.pa_read_idx = pa_i;
        self.pb_read_idx = pb_i;
        n
    }

    fn hard_resync(&mut self) {
        let r = self._tim.regs_gp16();
        // Stop the counter; no more ETR clocks → no more DMA.
        r.cr1().modify(|w| w.set_cen(false));
        // Clear any pending CC1IF / CC2IF / UIF status flags —
        // otherwise on restart the DMA channels can each consume
        // one stale interrupt flag as a "ghost" transfer,
        // re-introducing the pair drift we just resynced away.
        r.sr().modify(|w| {
            w.set_uif(false);
            w.set_ccif(0, false);
            w.set_ccif(1, false);
        });
        // Account a full ring's worth of drops (we can't tell
        // exactly how many were lost).
        let n = self.pa_ring.capacity() as u32;
        self.dropped = self.dropped.saturating_add(n);
        self.dropped_total = self.dropped_total.saturating_add(n);
        self.pa_ring.clear();
        self.pb_ring.clear();
        // Re-anchor our own read indices to the embassy ring's new
        // (cleared) write position. embassy's clear() sets read_index
        // to wherever the DMA currently is, so reading NDTR now gives
        // us the same anchor.
        let pa_ndtr = pac::DMA1.ch(4).ndtr().read().ndt() as usize;
        let pb_ndtr = pac::DMA1.ch(6).ndtr().read().ndt() as usize;
        self.pa_read_idx = (RING_CAP - pa_ndtr) & RING_MASK;
        self.pb_read_idx = (RING_CAP - pb_ndtr) & RING_MASK;
        // Restart the counter — next WR edge fires both DMAs
        // from a known-empty state.
        r.cr1().modify(|w| w.set_cen(true));
    }

    /// Wait until at least one paired sample is available, then
    /// fast-drain whatever's currently in both rings (up to
    /// `out.len()`) into packed u32 samples and return the count.
    ///
    /// Wakes on the DMA's half-/full-transfer IRQ (at N/2 and wrap)
    /// via embassy's waker, then bypasses embassy's ring read path
    /// and reads directly from the DMA buffer via `fast_drain`.
    pub async fn read_available(&mut self, out: &mut [u32]) -> usize {
        poll_fn(|cx| {
            // Register on PA only — PB fills in lockstep, so PA's
            // half-/full-transfer IRQ implies PB has data too.
            self.pa_ring.set_waker(cx.waker());
            let n = self.fast_drain(out);
            if n > 0 { Poll::Ready(n) } else { Poll::Pending }
        })
        .await
    }

    /// Take + reset the since-last-call dropped-samples counter.
    pub fn take_dropped(&mut self) -> u32 {
        core::mem::replace(&mut self.dropped, 0)
    }

    /// Lifetime cumulative dropped-samples count — never reset. For STATS.
    pub fn peek_dropped_total(&self) -> u32 {
        self.dropped_total
    }

    /// Read TIM2's current counter value. Useful as a sign-of-life
    /// check: if this isn't incrementing, ETR isn't seeing WR edges.
    pub fn peek_cnt(&self) -> u16 {
        self._tim.regs_gp16().cnt().read().cnt()
    }

    /// DEBUG: NDTR (remaining transfers) for the PA-ring DMA channel
    /// (TIM2_CH1 → DMA1_CH5) and PB-ring DMA channel (TIM2_CH2 →
    /// DMA1_CH7). If these decrement over time, DMA is firing.
    pub fn peek_dma_ndtr(&self) -> (u16, u16) {
        // BDMA channels are 1-indexed in hardware; ch(n) is 0-indexed.
        let pa_ndtr = pac::DMA1.ch(4).ndtr().read().ndt(); // CH5
        let pb_ndtr = pac::DMA1.ch(6).ndtr().read().ndt(); // CH7
        (pa_ndtr, pb_ndtr)
    }

    /// DEBUG: read + clear the DMA transfer-error flags for the PA/PB
    /// channels. TEIF is set if the DMA hit an AHB error mid-transfer
    /// — typically a bus arbitration loss the controller couldn't
    /// retry, or a peripheral handshake violation. Should be 0 in
    /// normal operation; nonzero means we lost samples and don't
    /// know how many. Returns `(pa_teif, pb_teif)`.
    pub fn take_dma_teif(&self) -> (bool, bool) {
        let isr = pac::DMA1.isr().read();
        let pa = isr.teif(4); // CH5
        let pb = isr.teif(6); // CH7
        // CTEIF is in the same bit positions in IFCR; writing a 1
        // clears the corresponding TEIF in ISR.
        if pa || pb {
            pac::DMA1.ifcr().write(|w| {
                if pa {
                    w.set_teif(4, true);
                }
                if pb {
                    w.set_teif(6, true);
                }
            });
        }
        (pa, pb)
    }
}
