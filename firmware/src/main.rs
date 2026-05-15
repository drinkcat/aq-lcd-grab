#![no_std]
#![no_main]

mod pio_capture;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::dma;
use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_time::Timer;
use panic_halt as _;

use pio_capture::{Capture, CapturePins, Sample};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
});

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"aq-lcd-grab capture PoC"),
    embassy_rp::binary_info::rp_program_description!(
        c"PIO+DMA capture of 8080 bus -> USB CDC dump"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(2048, log::LevelFilter::Info, driver);
}

// One-shot capture buffer. 4096 words ≈ one target burst with headroom.
const CAPTURE_LEN: usize = 4096;
static mut CAPTURE_BUF: [Sample; CAPTURE_LEN] = [0; CAPTURE_LEN];

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(driver).unwrap());

    Timer::after_secs(2).await;
    log::info!("aq-lcd-grab capture PoC starting");

    let pio = Pio::new(p.PIO0, Irqs);

    let mut capture = Capture::new(
        pio,
        p.DMA_CH0,
        CapturePins {
            db0: p.PIN_0,
            db1: p.PIN_1,
            db2: p.PIN_2,
            db3: p.PIN_3,
            db4: p.PIN_4,
            db5: p.PIN_5,
            db6: p.PIN_6,
            db7: p.PIN_7,
            db8: p.PIN_8,
            db9: p.PIN_9,
            db10: p.PIN_10,
            db11: p.PIN_11,
            db12: p.PIN_12,
            db13: p.PIN_13,
            db14: p.PIN_14,
            db15: p.PIN_15,
            dc: p.PIN_16,
            cs: p.PIN_17,
            wr: p.PIN_18,
        },
        Irqs,
    );

    loop {
        log::info!("waiting for {} samples on WR (GPIO 18)…", CAPTURE_LEN);

        // SAFETY: single-task access; DMA reads into it exclusively for the
        // duration of `await`, no aliasing.
        let buf = unsafe { &mut *core::ptr::addr_of_mut!(CAPTURE_BUF) };

        capture.capture(buf).await;

        log::info!("captured {} samples, first 32:", CAPTURE_LEN);
        for (i, &s) in buf.iter().take(32).enumerate() {
            log::info!(
                "  [{:4}] cs={} dc={} data=0x{:04x}",
                i,
                pio_capture::cs(s) as u8,
                pio_capture::dc(s) as u8,
                pio_capture::data(s),
            );
        }

        Timer::after_secs(2).await;
    }
}
