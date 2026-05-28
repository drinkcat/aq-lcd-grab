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

## Bench-rig wiring (target → Blue Pill F103C8)

The PCB and final firmware will use TIM1/PA12 for WR, but bench
development on a Blue Pill uses TIM2/PA0 because the Blue Pill ties
PA12 to 3V3 through a 1.5 kΩ USB-DP pull-up that distorts an
external WR signal.

**Two-DMA routing, GPIOB self-sufficient.** WR clocks the capture on
PA0 (fixed; TIM2 ETR). The two GPIO ports are read by separate DMA
channels and can develop a one-sample pairing skew under sustained
command bursts (the DMA arbiter services the two channels a cycle
apart, by which time the PIC32 in the target has advanced the bus — see
commit notes). To keep the rig useful even if one port's capture
fails, we pack **everything needed to decode the command stream plus
a colour image onto GPIOB alone**: DC (framing), the whole low data
byte DB0–DB7 (every command opcode + the low colour bits), and the
top two bits of each of red (R4/R3) and green (G5/G4) — the bits the
capture data shows carry the most colour weight. A GPIOB-only read
therefore decodes all commands *and* renders recognisable colour on
its own. **GPIOA carries WR, the remaining upper-byte bits** (G3 +
R0/R1/R2 — colour refinement that never holds command bits, so a
GPIOA-DMA skew only smears pixel colour) **and CS** (unused in
decode; DC frames the bus). Full fidelity needs both ports; GPIOB
alone is the graceful-degradation fallback.

SWD (PA13/PA14) cannot be remapped — the SW-DP pins are fixed in
silicon — but it *can* be disabled (AFIO `SWJ_CFG = disabled`),
freeing PA13/PA14 as plain GPIO. We do that here: the flash path is
the USART1 bootloader (PA9/PA10), so we don't need live SWD on the
bench rig.

Full GPIOA map (PA0–PA15) — WR + colour-refinement bits + CS. All
non-reserved pins are read together in the single GPIOA->IDR DMA;
"reserved" = not available for capture:

| F103 pin | Use            | target pin | Cable  | Notes                          |
|----------|----------------|---------|--------|--------------------------------|
| PA0      | WR (TIM2 ETR)  | 24      | green  | clocks the capture             |
| PA1      | DB8 (G3)       | 10      |        |                                |
| PA2      | DB11 (R0)      | 13      |        |                                |
| PA3      | DB12 (R1)      | 14      |        |                                |
| PA4      | DB13 (R2)      | 15      |        |                                |
| PA5      | CS             | 22      | orange | not used in decode; DC frames  |
| PA6      | —              | —       |        | free                           |
| PA7      | —              | —       |        | free                           |
| PA8      | —              | —       |        | free                           |
| PA9      | USART1 TX      | —       |        | reserved (flash + log path)    |
| PA10     | USART1 RX      | —       |        | reserved (flash + log path)    |
| PA11     | —              | —       |        | free                           |
| PA12     | —              | —       |        | reserved (USB-DP 1.5 kΩ pull-up) |
| PA13     | —              | —       |        | free (SWDIO; SWJ off). Reset pull-up. |
| PA14     | —              | —       |        | free (SWCLK; SWJ off). Reset pull-down. |
| PA15     | —              | —       |        | free (JTDI; SWJ off). Reset pull-up. |

Full GPIOB map (PB0–PB15) — the self-sufficient port: DC + low byte
DB0–DB7 + the top two red (R4/R3) and top two green (G5/G4) bits. In
pin order:

| F103 pin | Use         | target pin | Cable  | Notes                        |
|----------|-------------|---------|--------|------------------------------|
| PB0      | DB14 (R3)   | 16      |        | red                          |
| PB1      | DB15 (R4)   | 17      |        | red MSB                      |
| PB2      | —           | —       |        | reserved (BOOT1)             |
| PB3      | —           | —       |        | reserved (JTAG out at reset) |
| PB4      | —           | —       |        | reserved (JTAG out at reset) |
| PB5      | DB0         | 2       |        |                              |
| PB6      | DB1         | 3       |        |                              |
| PB7      | DB2         | 4       |        |                              |
| PB8      | DB3         | 5       |        |                              |
| PB9      | DB4         | 6       |        |                              |
| PB10     | DB5         | 7       |        |                              |
| PB11     | DB6         | 8       |        |                              |
| PB12     | DB7         | 9       |        |                              |
| PB13     | DB9 (G4)    | 11      |        | green                        |
| PB14     | DB10 (G5)   | 12      |        | green MSB                    |
| PB15     | DC          | 23      | yellow | framing signal               |

Each port's bit order is free — re-order for clean PCB routing as
long as `permute_f103` matches. PB2 (BOOT1) and PB3/PB4 (JTAG
outputs at reset) are avoided. GPIOB is now full (DC + DB0–DB7 + 4
colour bits); GPIOA has four free pins (PA11 and the ex-SWD trio
PA13–PA15).

Why this split: a GPIOB-only capture decodes the whole command
stream *and* renders recognisable colour on its own. DB0–DB7 carries
every opcode plus the low colour bits — across 1.88 M samples in
`host/goodrun/run.bin` the low byte alone distinguishes the dominant
fills (green, black, white, yellow, orange). Adding the top two bits
of red (R4/R3) and green (G5/G4) is what makes the fallback image
*coloured* rather than monochrome: those four bits are, per-channel,
the most colour-faithful pair (keeping R4/R3 gives the lowest
reconstruction error of any red pair, G5/G4 likewise for green).
GPIOA holds only the lower colour-refinement bits (G3, R0–R2) plus
the unused CS, so losing the GPIOA port costs colour depth, not
legibility or basic colour.

`host/src/permute.rs::permute_f103` knows this routing and
unscrambles the GPIOB sample into logical `(data_lo, dc)` plus the
high colour bits it carries, and the GPIOA sample into the remaining
colour-refinement bits.

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
