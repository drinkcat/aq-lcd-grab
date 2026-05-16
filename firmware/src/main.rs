#![no_std]
#![no_main]

mod decoder;
mod picotool_reset;
mod pio_capture;
mod proto;

use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_rp::bind_interrupts;
use embassy_rp::dma;
use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcState};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use panic_halt as _;

use decoder::{Decoder, Sample as DecSample, Transaction};
use picotool_reset::PicotoolHandler;
use pio_capture::{CapturePins, RingCapture, Sample as RawSample};

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
        c"PIO+DMA capture of 8080 bus -> binary CDC stream"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

// Ring buffer: power-of-2 sample count, byte-size also power-of-2,
// aligned to its size in bytes. 8192 samples × 4 B = 32768 B
// (ring_size=15, which is the RP2350 maximum). Each pixel of an 8080
// bus transfer is one sample, so this buys ~8K samples (~50 ms at 200
// kHz bus rate, or ~400 µs at 20 MHz) of slack while USB CDC drains.
const RING_LEN: usize = 8192;
#[repr(align(32768))]
struct RingBuf([RawSample; RING_LEN]);
static mut RING_BUF: RingBuf = RingBuf([0; RING_LEN]);

/// Sample chunk size we drain per loop iteration. Decode + send happens
/// in tight bursts of this many samples.
const DRAIN_CHUNK: usize = 1024;

/// Channel from the capture task to the USB sender.
type TxChannel = Channel<ThreadModeRawMutex, Transaction, 8>;
type TxTx<'a> = Sender<'a, ThreadModeRawMutex, Transaction, 8>;

static TX_CHANNEL: TxChannel = Channel::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let driver = Driver::new(p.USB, Irqs);

    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("aq-lcd-grab");
    config.product = Some("Capture PoC (binary)");
    config.serial_number = Some("aq-lcd-grab");
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    static mut CONFIG_DESC: [u8; 256] = [0; 256];
    static mut BOS_DESC: [u8; 256] = [0; 256];
    static mut MSOS_DESC: [u8; 256] = [0; 256];
    static mut CONTROL_BUF: [u8; 64] = [0; 64];
    static CDC_STATE: static_cell::StaticCell<CdcState> = static_cell::StaticCell::new();
    static PICOTOOL_HANDLER: static_cell::StaticCell<PicotoolHandler> =
        static_cell::StaticCell::new();

    let cdc_state = CDC_STATE.init(CdcState::new());
    let picotool_handler = PICOTOOL_HANDLER.init(PicotoolHandler::new());

    // SAFETY: each static-mut buffer is borrowed exactly once.
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

    let cdc_class = CdcAcmClass::new(&mut builder, cdc_state, 64);

    let iface_num = {
        let mut func = builder.function(0xFF, 0x00, 0x01);
        let mut iface = func.interface();
        let num = iface.interface_number();
        let _alt = iface.alt_setting(0xFF, 0x00, 0x01, None);
        num
    };
    picotool_handler.set_interface(iface_num);
    builder.handler(picotool_handler);

    let mut usb = builder.build();

    let pio = Pio::new(p.PIO0, Irqs);
    // SAFETY: RING_BUF is only ever borrowed here, exactly once, for the
    // remainder of the program.
    let ring_slice: &'static mut [RawSample] = unsafe {
        let ptr = core::ptr::addr_of_mut!(RING_BUF.0) as *mut RawSample;
        core::slice::from_raw_parts_mut(ptr, RING_LEN)
    };
    let mut capture = RingCapture::new(
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
        ring_slice,
    );

    let (mut sender, _receiver) = cdc_class.split();

    let usb_fut = usb.run();

    let sender_fut = async {
        sender.wait_connection().await;
        loop {
            let tx = TX_CHANNEL.receive().await;
            if let Err(_e) = send_frame(&mut sender, &tx).await {
                // Host disconnected; wait for reconnect.
                sender.wait_connection().await;
            }
        }
    };

    let capture_fut = async {
        Timer::after_millis(500).await;
        log_line(TX_CHANNEL.sender(), "ring capture starting");

        let mut decoder = Decoder::default();
        let mut chunk = [0u32; DRAIN_CHUNK];
        let mut idle_ticks: u32 = 0;
        loop {
            // Drain everything available before sleeping, not just one
            // DRAIN_CHUNK. The ring is much larger than the chunk, so
            // capping at DRAIN_CHUNK per polling tick would needlessly
            // give back our headroom under sustained traffic.
            let mut total = 0usize;
            loop {
                let n = capture.drain(&mut chunk);
                if n == 0 {
                    break;
                }
                total += n;
                for &raw in &chunk[..n] {
                    let sample = DecSample::from_raw(raw);
                    if let Some(tx) = decoder.feed(sample) {
                        TX_CHANNEL.send(tx).await;
                    }
                }
            }
            let dropped = capture.take_dropped();
            if dropped > 0 {
                let mut msg = heapless::String::<32>::new();
                use core::fmt::Write as _;
                let _ = write!(msg, "overrun: dropped {dropped}");
                // Use blocking send: the overrun message is the whole
                // point of the detection, and try_send silently drops
                // it exactly when the channel is full — which is when
                // overruns happen.
                let mut tx = Transaction::new(proto::CMD_LOG);
                for chunk in msg.as_bytes().chunks(2) {
                    let lo = chunk[0];
                    let hi = if chunk.len() > 1 { chunk[1] } else { 0 };
                    if tx.data.push(u16::from_le_bytes([lo, hi])).is_err() {
                        break;
                    }
                }
                TX_CHANNEL.send(tx).await;
            }
            if total == 0 {
                idle_ticks = idle_ticks.wrapping_add(1);
                if idle_ticks.is_multiple_of(2500) {
                    // Every ~5 s of pure idle, breadcrumb so the host
                    // can tell "no traffic" from "firmware wedged".
                    log_line(TX_CHANNEL.sender(), "idle");
                }
                Timer::after_millis(2).await;
            } else {
                idle_ticks = 0;
            }
        }
    };

    join3(usb_fut, sender_fut, capture_fut).await;
}

async fn send_frame<'d, D: embassy_usb::driver::Driver<'d>>(
    sender: &mut embassy_usb::class::cdc_acm::Sender<'d, D>,
    tx: &Transaction,
) -> Result<(), EndpointError> {
    // Heuristic: if this is a memory-write frame and all data words are
    // equal, send a single RLE pair (4 bytes) instead of the raw words.
    // Big background fills compress from ~8 KB per frame to ~9 bytes.
    let is_mw = tx.cmd == 0x2C || tx.cmd == proto::CMD_MEMORY_WRITE_CONTINUE;
    let uniform = is_mw
        && !tx.data.is_empty()
        && tx.data.len() <= u16::MAX as usize
        && tx.data.iter().all(|&w| w == tx.data[0]);

    // CDC packet size is 64. We coalesce header + payload, then chunk.
    let mut buf = [0u8; 64];
    let mut fill = 0;

    macro_rules! push {
        ($byte:expr) => {{
            buf[fill] = $byte;
            fill += 1;
            if fill == 64 {
                sender.write_packet(&buf).await?;
                fill = 0;
            }
        }};
    }

    let payload_bytes: usize;
    if uniform {
        // RLE encoding: 1 pair = 2 words = 4 bytes.
        let header =
            proto::encode_header(tx.cmd, 2u16 | proto::RLE_FLAG);
        for &b in &header {
            push!(b);
        }
        let run_len = tx.data.len() as u16;
        let value = tx.data[0];
        for &w in &[run_len, value] {
            push!((w & 0xFF) as u8);
            push!((w >> 8) as u8);
        }
        payload_bytes = 4;
    } else {
        let count = tx.data.len() as u16;
        let header = proto::encode_header(tx.cmd, count);
        for &b in &header {
            push!(b);
        }
        for &w in &tx.data {
            push!((w & 0xFF) as u8);
            push!((w >> 8) as u8);
        }
        payload_bytes = 2 * count as usize;
    }

    if fill > 0 {
        sender.write_packet(&buf[..fill]).await?;
    }

    // USB CDC: a transfer ending exactly on a packet boundary needs a
    // ZLP to terminate. Avoid the ambiguity by emitting a short packet
    // whenever the total length is a multiple of 64.
    if (5 + payload_bytes).is_multiple_of(64) {
        sender.write_packet(&[]).await?;
    }

    Ok(())
}

fn log_line(ch: TxTx<'_>, msg: &str) {
    let mut tx = Transaction::new(proto::CMD_LOG);
    // Pack two ASCII chars per u16, LE. Trailing odd byte gets a NUL.
    let bytes = msg.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        let lo = bytes[i];
        let hi = if i + 1 < bytes.len() { bytes[i + 1] } else { 0 };
        if tx.data.push(u16::from_le_bytes([lo, hi])).is_err() {
            break;
        }
        i += 2;
    }
    let _ = ch.try_send(tx);
}

