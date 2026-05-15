//! PIO program for capturing the target device's 16-bit 8080 MCU bus.
//!
//! Pin assignment (consecutive GPIOs required for `in pins, N`):
//!
//!     GPIO 0..=15 -> DB0..=DB15  (16-bit data bus)
//!     GPIO 16     -> D/C (RS)
//!     GPIO 17     -> CS
//!     GPIO 18     -> WR (write strobe — sample trigger)
//!
//! Each captured word in the RX FIFO is laid out (LSB first):
//!
//!     bit  17 16 15 ............... 0
//!          CS DC DB15 ............ DB0
//!
//! Upper 14 bits of the 32-bit word are zero (autopush threshold = 18).
//!
//! Sampling on WR rising edge — 8080 spec says data is valid then, and the
//! display is the slave being driven by the MCU, so the rising edge marks
//! "data has just been latched, sample it now".

use embassy_rp::Peri;
use embassy_rp::dma::{self, Channel, Transfer};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{Common, Config, Pio, ShiftConfig, ShiftDirection, StateMachine};

/// One sample as it lands in the FIFO. Lower 18 bits are live signals.
pub type Sample = u32;

/// Extract the 16 data bits.
#[inline]
#[allow(dead_code)]
pub fn data(s: Sample) -> u16 {
    (s & 0xFFFF) as u16
}

/// Extract D/C (1 = data, 0 = command).
#[inline]
#[allow(dead_code)]
pub fn dc(s: Sample) -> bool {
    (s >> 16) & 1 != 0
}

/// Extract CS (1 = deasserted, 0 = asserted).
#[inline]
#[allow(dead_code)]
pub fn cs(s: Sample) -> bool {
    (s >> 17) & 1 != 0
}

pub struct Capture<'d> {
    // Keep `Common` alive — dropping it reverts every claimed pin's funcsel
    // to NULL, which silently breaks the running state machine.
    _common: Common<'d, PIO0>,
    sm: StateMachine<'d, PIO0, 0>,
    dma: Channel<'d>,
}

impl<'d> Capture<'d> {
    pub fn new<DmaCh>(
        pio: Pio<'d, PIO0>,
        dma_peri: Peri<'d, DmaCh>,
        pins: CapturePins<'d>,
        dma_irqs: impl embassy_rp::interrupt::typelevel::Binding<
            <DmaCh as dma::ChannelInstance>::Interrupt,
            dma::InterruptHandler<DmaCh>,
        > + 'd,
    ) -> Self
    where
        DmaCh: dma::ChannelInstance,
    {
        let Pio {
            mut common,
            mut sm0,
            ..
        } = pio;

        let prg = pio_asm!(
            ".wrap_target",
            "    wait 0 gpio 18", // WR goes low (host starts writing)
            "    wait 1 gpio 18", // WR returns high — data is latched, sample now
            "    in pins, 18",    // sample {CS, DC, DB15..DB0}
            ".wrap",
        );

        let loaded = common.load_program(&prg.program);

        // Claim the input pins for the PIO. They stay as inputs.
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
            // Autopush after 18 bits — every WR rising edge emits one FIFO word.
            auto_fill: true,
            threshold: 18,
            // ISR shifts left: `in pins, 18` puts pin_base+0 (DB0) at ISR bit 0,
            // pin_base+17 (CS) at ISR bit 17. (With Right, bits land in 14..31
            // of the 32-bit pushed word — confusing for downstream decode.)
            direction: ShiftDirection::Left,
        };
        // Default clock divider = 1 -> PIO runs at sys_clk (150 MHz on RP2350).
        // WR period is ~1.5 µs, our loop body is 3 instructions = 20 ns. Plenty.

        sm0.set_config(&cfg);
        sm0.set_enable(true);

        Self {
            _common: common,
            sm: sm0,
            dma: Channel::new(dma_peri, dma_irqs),
        }
    }

    /// One-shot capture: fill `buf` with N consecutive samples, then return.
    /// On the real target a "burst" is ~3300 samples (~6.6 KB) every ~1 s, so
    /// 4096 words is a comfortable single-burst capture.
    pub fn capture<'a>(&'a mut self, buf: &'a mut [Sample]) -> Transfer<'a> {
        self.sm.rx().dma_pull(&mut self.dma, buf, false)
    }
}

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
