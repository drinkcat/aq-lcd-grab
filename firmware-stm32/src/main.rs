#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{Config as UsartConfig, Uart};
use embassy_stm32::{Config, bind_interrupts, peripherals, usart};
use embassy_time::Timer;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // HSI (8 MHz) -> /2 -> PLL x16 -> 64 MHz SYSCLK. No external
    // crystal, per pcb_spec.md "Clocking". On F1 the HSI->PLL path is
    // hardwired /2 and embassy-stm32 hard-panics if PllPreDiv != DIV2.
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

    // PC13 — onboard LED on Blue Pill, status LED on the capture PCB.
    let mut led = Output::new(p.PC13, Level::High, Speed::Low);

    let mut usart_cfg = UsartConfig::default();
    usart_cfg.baudrate = 115200;
    let mut usart = Uart::new(
        p.USART1,
        p.PA10, // RX
        p.PA9,  // TX
        Irqs,
        p.DMA1_CH4, // TX DMA — DMA1 Ch4 per RM0008 Table 78.
        p.DMA1_CH5, // RX DMA — DMA1 Ch5 (capture path will need this
        // channel for TIM1_UP later; move USART RX to interrupt mode
        // then, since RX traffic is sparse).
        usart_cfg,
    )
    .unwrap();

    let blink = async {
        loop {
            led.toggle();
            Timer::after_millis(500).await;
        }
    };

    let hello = async {
        let mut n: u32 = 0;
        let mut buf = [0u8; 64];
        loop {
            let len = format_hello(&mut buf, n);
            let _ = usart.write(&buf[..len]).await;
            n = n.wrapping_add(1);
            Timer::after_millis(1000).await;
        }
    };

    join(blink, hello).await;
}

fn format_hello(buf: &mut [u8], n: u32) -> usize {
    const PREFIX: &[u8] = b"hello from stm32f103 #";
    const SUFFIX: &[u8] = b"\r\n";
    let mut i = 0;
    for &b in PREFIX {
        buf[i] = b;
        i += 1;
    }
    let mut tmp = [0u8; 10];
    let mut t = 0;
    let mut v = n;
    if v == 0 {
        tmp[0] = b'0';
        t = 1;
    } else {
        while v > 0 {
            tmp[t] = b'0' + (v % 10) as u8;
            v /= 10;
            t += 1;
        }
    }
    while t > 0 {
        t -= 1;
        buf[i] = tmp[t];
        i += 1;
    }
    for &b in SUFFIX {
        buf[i] = b;
        i += 1;
    }
    i
}
