#![no_std]
#![no_main]

mod picotool_reset;
mod pio_capture;

use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_rp::bind_interrupts;
use embassy_rp::dma;
use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcState};
use embassy_usb::{Builder, Config};
use panic_halt as _;

use picotool_reset::PicotoolHandler;
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
        c"PIO+DMA capture of 8080 bus -> USB CDC dump; picotool-reset enabled"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

const CAPTURE_LEN: usize = 4096;
static mut CAPTURE_BUF: [Sample; CAPTURE_LEN] = [0; CAPTURE_LEN];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let driver = Driver::new(p.USB, Irqs);

    // ------------------------------------------------------------------
    // USB device setup: CDC ACM for `log::info!` output + a vendor-spec
    // interface so `picotool reboot -f -u` can put us into BOOTSEL.
    // ------------------------------------------------------------------
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("aq-lcd-grab");
    config.product = Some("Capture PoC + picotool-reset");
    config.serial_number = Some("aq-lcd-grab");
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    // Composite device: signal that interfaces share a function.
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Buffers must outlive the Builder; embassy-usb stashes them and
    // returns descriptors that reference them.
    static mut CONFIG_DESC: [u8; 256] = [0; 256];
    static mut BOS_DESC: [u8; 256] = [0; 256];
    static mut MSOS_DESC: [u8; 256] = [0; 256];
    static mut CONTROL_BUF: [u8; 64] = [0; 64];
    static CDC_STATE: static_cell::StaticCell<CdcState> = static_cell::StaticCell::new();
    static PICOTOOL_HANDLER: static_cell::StaticCell<PicotoolHandler> =
        static_cell::StaticCell::new();

    let cdc_state = CDC_STATE.init(CdcState::new());
    let picotool_handler = PICOTOOL_HANDLER.init(PicotoolHandler::new());

    // SAFETY: the four `static mut` buffers are only ever borrowed here,
    // exactly once, for the duration of the program. Embassy's USB stack
    // owns the references until the device is dropped (never, in this
    // firmware).
    let (config_desc, bos_desc, msos_desc, control_buf) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(CONFIG_DESC),
            &mut *core::ptr::addr_of_mut!(BOS_DESC),
            &mut *core::ptr::addr_of_mut!(MSOS_DESC),
            &mut *core::ptr::addr_of_mut!(CONTROL_BUF),
        )
    };

    let mut builder = Builder::new(
        driver,
        config,
        config_desc,
        bos_desc,
        msos_desc,
        control_buf,
    );

    // Logger CDC class.
    let cdc_class = CdcAcmClass::new(&mut builder, cdc_state, 64);

    // picotool reset interface — class 0xFF / subclass 0x00 / protocol 0x01.
    let iface_num = {
        let mut func = builder.function(0xFF, 0x00, 0x01);
        let mut iface = func.interface();
        // Interface number is allocated when we call .interface().
        let num = iface.interface_number();
        // Alt setting required by the descriptor; no endpoints.
        let _alt = iface.alt_setting(0xFF, 0x00, 0x01, None);
        num
    };
    picotool_handler.set_interface(iface_num);
    builder.handler(picotool_handler);

    let mut usb = builder.build();

    // ------------------------------------------------------------------
    // PIO+DMA capture.
    // ------------------------------------------------------------------
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

    // ------------------------------------------------------------------
    // Three concurrent futures: USB pump, logger pump, capture loop.
    // ------------------------------------------------------------------
    let usb_fut = usb.run();
    let log_fut = embassy_usb_logger::with_class!(2048, log::LevelFilter::Info, cdc_class);
    let capture_fut = async {
        Timer::after_secs(2).await;
        log::info!("aq-lcd-grab capture PoC starting (picotool-reset enabled)");

        loop {
            log::info!("waiting for {} samples on WR (GPIO 18)…", CAPTURE_LEN);

            // SAFETY: single-task access; DMA reads into it exclusively for
            // the duration of `await`, no aliasing.
            let buf = unsafe { &mut *core::ptr::addr_of_mut!(CAPTURE_BUF) };

            capture.capture(buf).await;

            log::info!("captured {} samples, dumping all:", CAPTURE_LEN);
            // Pack 4 samples per line: "[NNNN] cs=X dc=Y 0xHHHH ..." gets
            // long; instead use a compact hex layout "[NNNN] HHHHH HHHHH ..."
            // where each 5-hex-digit chunk encodes {CS bit 16, DC bit 15..0}.
            // 8 samples per line ≈ 70 chars, 4096/8 = 512 lines.
            for chunk_idx in (0..CAPTURE_LEN).step_by(8) {
                let mut line = heapless::String::<128>::new();
                let _ = core::fmt::write(&mut line, format_args!("[{:04}]", chunk_idx));
                for j in 0..8 {
                    let s = buf[chunk_idx + j];
                    // 18 bits total -> 5 hex digits.
                    let _ = core::fmt::write(&mut line, format_args!(" {:05x}", s & 0x3FFFF));
                }
                log::info!("{}", line.as_str());
                // Yield every 16 lines so the logger pipe drains.
                if (chunk_idx / 8) % 16 == 15 {
                    Timer::after_millis(20).await;
                }
            }
            log::info!("dump done");

            Timer::after_secs(3).await;
        }
    };

    join3(usb_fut, log_fut, capture_fut).await;
}
