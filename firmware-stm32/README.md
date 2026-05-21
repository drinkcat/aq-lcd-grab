# aq-lcd-grab — STM32F103 firmware (prototype)

Hello-world bring-up for the STM32F103C8T6 capture MCU described in
[`../docs/pcb_spec.md`](../docs/pcb_spec.md). Runs on the eventual
capture PCB but is being developed against a Keil MCBSTM32 dev board
(F103RBT6 — same Cortex-M3, same peripherals, more flash/pins).

## What it does

- **PC13 LED blink** at 1 Hz (matches the capture PCB's status LED and
  the Blue/Black/MCBSTM32 onboard LED).
- **USART1 @ 115200 8N1** on PA9 (TX) / PA10 (RX), emitting
  `hello from stm32f103 #N\r\n` once per second.
- 64 MHz from HSI + PLL, no external crystal — matches the no-crystal
  decision in the spec.

## Build

```sh
cargo build --release
```

Target (`thumbv7m-none-eabi`) is set in [.cargo/config.toml](.cargo/config.toml).
Install once with `rustup target add thumbv7m-none-eabi`.

## Flash (USART1 ROM bootloader)

Requires a USB-UART adapter (FT232 / CH340 / CP2102 etc.) and
`stm32flash` installed.

### Wiring

| USB-UART | STM32 pin | Note                          |
|----------|-----------|-------------------------------|
| TX       | PA10      | USART1 RX                     |
| RX       | PA9       | USART1 TX                     |
| GND      | GND       |                               |
| —        | BOOT0     | Tie **high** to enter loader  |
| —        | NRST      | Pulse low after BOOT0 is high |

### Procedure

1. Set BOOT0 high (jumper or tie to 3V3).
2. Pulse NRST low (button on the dev board).
3. `cargo run --release` — the runner shells out to
   [`scripts/flash-uart.sh`](scripts/flash-uart.sh), which converts the
   ELF to .bin and calls `stm32flash`.
4. Return BOOT0 low, pulse NRST. User firmware now runs.

Default port `/dev/ttyUSB0`, default baud 115200. Override:

```sh
STM32FLASH_PORT=/dev/ttyUSB1 STM32FLASH_BAUD=230400 cargo run --release
```

## Watch the UART output

```sh
# In another terminal, before/after flashing:
picocom -b 115200 /dev/ttyUSB0    # or screen, minicom, tio, etc.
```

If you used the same adapter for flashing, close `stm32flash` first
(the runner exits cleanly) before opening the terminal.

## Notes for the capture PCB vs the dev board

The MCBSTM32 dev board is an F103RBT6 (LQFP-64, 128 KB flash). The
target chip in [`../docs/pcb_spec.md`](../docs/pcb_spec.md) is
F103C8T6 (LQFP-48, 64 KB flash). The Cargo feature pinned in
[`Cargo.toml`](Cargo.toml) is `stm32f103c8` — `memory.x` is emitted
by `embassy-stm32`'s `memory-x` feature against the C8's 64 KB cap.
This keeps firmware size honest for the smaller production part; the
code runs unchanged on the RB.

PC13 lights an LED on both boards.

## Status

Stage 0: blink + UART hello. The TIM1 + dual-DMA capture path
described in [`../docs/pcb_spec.md`](../docs/pcb_spec.md) §"Capture
mechanism" is the next milestone.
