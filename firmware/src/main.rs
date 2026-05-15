#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::Timer;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"aq-lcd-grab hello"),
    embassy_rp::binary_info::rp_program_description!(c"Embassy hello world on Pico 2 W"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(driver).unwrap());

    // GPIO 25 isn't the onboard LED on Pico 2 W (that one's behind CYW43),
    // but we toggle it anyway for an optional external LED.
    let mut led = Output::new(p.PIN_25, Level::Low);

    let mut counter: u32 = 0;
    loop {
        counter = counter.wrapping_add(1);
        led.toggle();
        log::info!("hello from pico 2 w — tick {}", counter);
        Timer::after_millis(500).await;
    }
}
