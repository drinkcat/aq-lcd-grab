# target device Capture PCB — Design Spec

Working spec for an inline capture board: sits between the target device
main board and its LCD via two 39-pin flex connectors, taps the
display bus into an STM32F103C8T6 for capture, and hosts a Xiao
ESP32-C6 for WiFi-side processing and as the STM32's program loader.

Status: draft. Decisions captured below; open questions at the bottom.

**Capture MCU history:** the prototype was a Pico 2 W (RP2350) using
PIO + DMA. The PCB design briefly targeted a bare RP2350 in flashless
UART-boot mode, then was redrawn for STM32F030C8T6 (JLCPCB basic
library, internal flash → drops UART-boot complication), then refined
to **STM32F103C8T6** because the F103 is also JLC basic-library at
~$0.10–0.20 more, with 20 kB SRAM vs 8 kB and a Cortex-M3 vs M0 — a
material reduction in bring-up risk for the same form factor. See
"Addendum: STM32F030C8T6 considered, rejected" below.

## Overview

```
                  +-------------------+
   target main    |    Capture PCB    |     target LCD
   board          |                   |
        flex 39p  |  STM32F103C8T6    |  flex 39p
   ============== | passthrough + tap |==============
                  |                   |
                  |  Xiao ESP32-C6    |
                  |  (loader + WiFi)  |
                  +-------------------+
                            |
                       3-pin to main
                       board (3V3/GND/
                       PIC32 reset)
```

## Decisions

### MCU: STM32F103C8T6

- **JLCPCB basic-library part** — no extended-part assembly fee, no
  reel cost, always in stock. The primary reason for choosing STM32
  over RP2350.
- **Package:** LQFP-48, 7×7 mm body (9×9 mm with leads). Larger
  footprint than the RP2350 QFN-60, but no external SMPS inductor,
  polarised cap, or QSPI strap network is needed, so net board
  area is comparable.
- **Resources:** Cortex-M3 @ 72 MHz, 64 kB flash, **20 kB SRAM**,
  7 DMA channels, USB device, USART1 ROM bootloader on PA9/PA10.
  Pin-compatible with the F030C8T6 we briefly considered (same
  LQFP-48 pinout, same GPIO/USART/TIM1 layout for the pins we
  use).
- **Clocking:** internal HSI (8 MHz) × PLL → 64 MHz max on HSI/PLL
  (DS5319 Table 7 note 1). **No external crystal needed**: AN2606
  confirms the F103 ROM bootloader auto-bauds on HSI; we don't use
  USB (which would require HSE for the ±0.25% clock spec). HSI
  accuracy at 0–70 °C is ±1.3% to ±2% (DS5319 Table 24) — well
  within UART tolerance (~±2% per char at 8N1) at our baud rates.
- **Power:** single 3V3 rail. VDD/VDDA both 3V3 with 100 nF per pin
  + a 4.7 µF bulk near VDDA. No SMPS, no ferrite bead.

#### Addendum: STM32F030C8T6 considered, rejected

The F030C8T6 was the first STM32 picked for this board — same JLC
basic-library status, ~$0.10–0.20 cheaper. Switched to F103C8T6
because:

- **20 kB SRAM (F103) vs 8 kB (F030).** Removes the buffer-sizing
  risk that the burst-mode capture would not fit alongside an
  Embassy executor + USART buffers. Made Q15 trivially answerable.
- **Cortex-M3 vs M0.** M3 has DWT cycle-counter for bring-up
  timing measurements, bit-banding, and ~1.5× the integer
  throughput at 72 MHz vs 48 MHz. Reduces the bring-up risk that
  the ISR draining the DMA ring can't keep up.
- **Ecosystem.** F103 is the most-supported STM32 in existence
  (Blue Pill heritage). Faster iteration on examples and gotchas.
- **Pin-compatible.** Same LQFP-48, same usable AFs for the pins
  we care about (PA9/PA10 USART1, PA12 TIM1_ETR, PA13/PA14 SWD).
  The pin map below transfers verbatim from the F030 draft.
- **DMA channel mapping differs** (F103 has 7 channels vs F030's
  5; TIM1_UP is on Ch5, TIM1_CH1 on Ch2, TIM1_CH2 on Ch3 — see
  Q12). USART1 conflicts shift to different channels but the
  mitigation (run TX in interrupt mode, or remap) is the same
  shape as on F030.
- **Same crystal-free clocking.** Neither chip needs HSE for our
  use case.

Net cost delta ≈ +$0.10–0.20 per board for a meaningful reduction
in bring-up risk. The earlier F030-specific text below documents
the alternative for future reference.

#### Capture path: TIM1 + multi-channel DMA from GPIOx→IDR

Replaces the RP2350 PIO + DMA-ring approach. The target's WR strobe runs
at ~667 kHz (1.5 µs period, ~500 ns low pulse) — well within F103's
DMA throughput (and was already comfortable on the F030 draft).

##### Pin constraints

Same on F103 as on F030 — both LQFP-48 STM32s land the same
peripherals on the same pins for what we use. Verified against:

- F103: DS5319 Table 5 (pin definitions) / RM0008 Table for AF.
- F030: DS024849 Tables 11/12/13 (already consulted; identical
  mapping for our pins, see addendum above).

The naive "PA0–PA15 = DB0–DB15" plan is **not viable** because:

- **PA12 = TIM1_ETR.** Need PA12 free for the capture trigger.
- **PA13/PA14 = SWDIO/SWCLK** (default reset state). Repurposing
  them removes the ability to attach an ST-Link for bring-up.
- **PA9/PA10 = USART1 default** for the ROM bootloader. AN2606
  documents PA9/PA10 as the USART1 bootloader pins for both
  F030 and F103; PA9/PA10 must stay USART1-capable for
  ESP32-driven flashing.

So PA0–PA11 are usable for capture (with PA9/PA10 multifunction —
see below), and PA12–PA15 are reserved for ETR + SWD + USART1
infrastructure.

##### Pin allocation (rules, not a fixed map)

The capture path reads `GPIOA->IDR` and `GPIOB->IDR` as whole 16-bit
ports via two DMA channels — so which display-bus bit lands on which
physical pin **within a given port** is up to the router. A
permutation table on the host decoder un-shuffles them once per
captured event; cost is negligible because the decoder already
exists in software.

What must be true (hard constraints):

- **PA12 = WR** (TIM1_ETR; no remap on F103).
- **PA9 = USART1_TX, PA10 = USART1_RX**: permanent, single
  ESP32 ↔ STM32 link, used both for ROM-bootloader flashing
  (AN2606) and runtime data.
- **PA13 = SWDIO, PA14 = SWCLK**: keep for debug.
- **NRST and BOOT0** on their dedicated pins (LQFP-48 pins 7 and
  44 respectively).
- **PB2 = BOOT1 latch**: tie to GND through 10 kΩ, do not drive
  with firmware, do not use as an output.
- **DC and CS** on any free pin on either port. Both `GPIOA->IDR`
  and `GPIOB->IDR` are read every WR edge by parallel DMA channels
  (Q12), so a captured pin on either port is decodable in software.
- **Status LED** on any free pin outside the boot-strap pins.
  Convention: PC13, matching the Blue/Black Pill onboard LED so the
  same firmware blinks both dev board and PCB.

Pin assignments to fix in routing (flexible):

- **Data bus (DB0–DB15)** must use 8 pins on PA + 8 pins on PB.
  - PA-side bits: pick 8 pins from PA0–PA8, PA11, PA15 (excluding
    PA9/PA10/PA12/PA13/PA14 and PA8/PA11/PA15 only if free of JTAG
    in your firmware config). PA0–PA8 + PA11 is the natural 10-
    candidate pool — pick whichever 8 the router likes best.
  - PB-side bits: pick 8 pins from PB0–PB15 excluding PB2 (BOOT1).
    Any 8 of the remaining 15 are fine — the host permute step
    doesn't care which bit positions within `GPIOB->IDR` we use,
    so high-byte vs low-byte alignment doesn't matter.
- The logical "DB0..DB15" identity of each pin is **defined by
  the SKiDL net names**, not by physical location. Routing can
  shuffle freely within each port; the host decoder will pick up
  the mapping from a single `LOGICAL_TO_PHYSICAL[16]` table.

Suggested starting allocation (used for area estimation in Q8;
swap freely during routing):

| Signal     | Pin (suggested) | Notes                                  |
|------------|-----------------|----------------------------------------|
| DB-low ×8  | PA0–PA7         | LQFP-48 pins 10–17, one clean edge     |
| DB-high ×8 | PB8–PB15        | LQFP-48 pins 21–22, 25–28, 45–46       |
| DC, CS     | PB0, PB1        | any free pin on either port (Q12)      |
| LED        | PC13            | matches Blue/Black Pill onboard LED    |

The header [`firmware/src/pio_capture.rs`](../firmware/src/pio_capture.rs)
shows the prototype's bit-to-pin mapping for the Pico 2 W — it's
purely informational for the STM32 design; the host's bus decoder
([`host/src/bus_decoder.rs`](../host/src/bus_decoder.rs)) consumes
permuted samples from the per-board permutation table in
[`host/src/permute.rs`](../host/src/permute.rs) — that table is the
piece that grows an F103 entry alongside the existing Pico one.

##### Capture mechanism (2 DMA channels, F103-verified)

TIM1 in **external clock mode 2** (ECE=1 in TIM1_SMCR), clocked by
ETR (= WR). With **ARR=0**, every ETR edge increments the counter
to 0 and triggers an update event (UEV); a CC1 channel in
output-compare mode with CCR=0 generates a compare-match event on
the same cycle.

Each TIM1 event has an independent DMA request line, enabled via
the DIER register: UDE, CC1DE, etc. The line-to-channel routing
is in RM0008 Table 78 for F103 (different on F030 — RM0360 Table
26):

| TIM1 source | DMA1 channel | Notes                            |
|-------------|--------------|----------------------------------|
| TIM1_CH1    | Ch 2         | reads `GPIOA->IDR` → data-PA ring   |
| TIM1_UP     | Ch 5         | reads `GPIOB->IDR` → data-PB + ctrl ring |
| TIM1_CH3    | Ch 6         | optional: counter snapshot ring  |

Both primary channels fire on the same WR edge because TIM1's
overflow simultaneously produces UEV and the CH1 compare-match
event.

**Ch 5 conflict:** Ch 5 is also `USART1_RX`'s default DMA channel,
and F103 has no DMA remap. Run USART1 RX interrupt-driven instead —
RX carries only sparse control messages from the ESP32 (the heavy
direction is STM32→ESP32 streaming captured bursts), so even at
1 Mbaud the ISR load is negligible.

**Hardware glitch filter free with ETR:** ETR has a built-in
digital filter (ETF[3:0] in TIM1_SMCR), so we can replicate the
RP2350 PIO's `wait 0 gpio 18 [2]` glitch-reconfirm in hardware by
setting ETF to require N consecutive samples at the new level
before propagating ETRF. The Pico 2 W firmware filtered glitches
in software ([`pio_capture.rs`](../firmware/src/pio_capture.rs)
line ~98); we get equivalent filtering for free.

**Source-vs-destination pointer convention:** with the source set
to `&GPIOx->IDR` (16-bit half-word), `PINC=0` (source no
increment), `MINC=1` (destination increment), `MEM2MEM=0`
(peripheral mode — wait for HW request), the channel waits for its
peripheral request from TIM1 and on each request copies one
half-word from `GPIOx->IDR` to the next ring slot. Wrap by setting
the destination buffer to a power-of-2-aligned span and reloading
the counter via circular mode (`CIRC=1`).
- **Why DC capture matters:** the protocol decoder uses DC as the
  command/data framing line (see
  [`host/src/bus_decoder.rs`](../host/src/bus_decoder.rs)).
  Without it, we can't tell which words are command bytes (0x2A,
  0x2B, 0x2C…) vs RGB565 pixel data. Capturing it on a second DMA
  channel is the F030 equivalent of what the RP2350 PIO did by
  shifting `{CS, DC, DB15..DB0}` into a single 18-bit FIFO entry.
- **Mechanism:** TIM1 in slave mode, clocked by ETR (WR). On every
  WR edge (configurable rising/falling — current PIO firmware uses
  falling-edge sampling with a 2-cycle reconfirm filter), TIM1
  update event triggers two DMA channels in parallel: channel A
  copies `GPIOA->IDR` (data bus) and channel B copies `GPIOB->IDR`
  (control bus, mask off in software) into their respective rings.
  CPU is uninvolved per-sample; firmware processes bursts at the
  boundaries detected by an idle-period timer.
- **Ring buffer sizing:** average burst is ~3,300 × 2 bytes (data) +
  ~3,300 × 2 bytes (control) ≈ 13 kB. F103's 20 kB SRAM holds a full
  burst with room for stack + USART buffers + Embassy executor, so
  whole-burst buffering is the default; mid-burst USART streaming
  is available as a fallback if any future use shifts the average
  burst higher.
- **DMA throughput risk:** ~667 kHz × 2 channels of half-word
  GPIO→SRAM transfers. At 72 MHz that's ~108 cycles per transfer pair
  — comfortable on the F103's bus matrix even with CPU active. Still
  the new front-end risk vs the RP2350 PIO prototype, so validate
  with a no-drop-counter check during bring-up.

#### Programming path: SWD + ESP32-driven UART bootloader

Both paths are wired so we can fall back if either breaks.

- **SWD header** (3-pin: SWDIO/SWCLK/GND, plus 3V3 from board) on
  PA13/PA14. Drives bring-up with an ST-Link or Pi Pico probe.
  Standard 1.27 mm pitch SWD or 2.54 mm DIP header — decide at
  layout time based on board space.
- **ESP32 UART bootloader path:** ESP32 drives `BOOT0` high and
  pulses `NRST`, then talks to USART1 on PA9 (RX) / PA10 (TX) at
  up to 115200 baud. BOOT0 line and NRST line from ESP32 GPIOs.
  Pull-down on BOOT0 so normal boots run user flash.
- **No QSPI strap network, no UART-boot SRAM-image dance, no 6 s
  boot latency.** Firmware lives in STM32 internal flash and
  persists across power cycles.

##### Firmware-skew risk and how we manage it

With the STM32 holding its own firmware in internal flash,
RP2350's "single source of truth" property is lost. Mitigations:

- ESP32 stores the canonical STM32 firmware image alongside its
  own in a dedicated partition.
- On boot, ESP32 reads STM32 version (over runtime UART — see
  below) and reflashes via the bootloader path if it doesn't
  match.
- This is the same OTA pattern as the RP2350 plan, just with
  STM32-bootloader semantics instead of bootrom UART boot.

#### Runtime UART (after firmware boots)

- USART1 (PA9/PA10) doubles as the runtime ESP32 ↔ STM32 link, same
  pins as the bootloader. No re-muxing required (unlike the RP2350
  plan which switched from QSPI bootrom UART to PL011 F11 mux).
- Baud rate negotiable between firmwares; default 115200 or higher.

### Power: tap target 3V3 rail

- Single 3V3 rail powers the whole capture board (STM32 + Xiao C6).
- Backfed into Xiao C6 3V3 pin, which bypasses its onboard LDO. This
  means the Xiao cannot be powered from its own USB-C while the
  capture PCB is connected to the target.
- **To reflash the ESP32 standalone:** disconnect the 3V3 pin in the
  3-wire connector to the target main board so the Xiao can run from
  its own USB-C without back-driving the rail.
- **Risk to validate:** Xiao C6 WiFi TX pulls 200–300 mA peaks. target
  3V3 must source that on top of the display backlight load. Measure
  the target 3V3 headroom before committing.
- **Fallback if 3V3 headroom is insufficient:** a 2-pin 2.54 mm header
  (J4, unpopulated by default) lets us feed target 5V into the Xiao
  USB-C VBUS net, so the Xiao's onboard LDO regenerates 3V3 locally
  instead of leaning on the target 3V3 rail. Don't have both Xiao
  USB-C and the J4 tap live at the same time without an OR'ing
  scheme — they'd fight on the +5V rail.

#### Decoupling

STM32 local decoupling per AN2586 §2.2
([`reference/STM32F103/an2586_hardware_development.pdf`](../reference/STM32F103/an2586_hardware_development.pdf)):

| Rail              | Caps                                          |
|-------------------|-----------------------------------------------|
| VDD (×3 on LQFP-48) | 100 nF per pin + 1× 4.7 µF bulk             |
| VDDA              | 100 nF + 1 µF                                 |
| VBAT (no battery) | tied to VDD via 100 nF                        |

VREF+ is not bonded out on the F103C8 LQFP-48 package (DS5319
Table 5 — only on LFBGA100 / TFBGA64); no external decoupling
needed.

Plus 22 µF + 100 µF bulk near the Xiao 3V3 pad to absorb WiFi TX
transients locally rather than burdening the target regulator's
transient response (see Q6 mitigation).

### Connection to target main board

- 3-pin connector on the capture PCB: 3V3 (power tap), GND, PIC32
  reset. The target main board exposes a 5-pin header but we only
  need these three.
- PIC32 reset is open-drain with pull-up on the main board (confirmed
  by user from prior project).
- ESP32 drives it open-drain: GPIO as input = released, GPIO low =
  asserted. **Never drive high.**

### Display capture tap

- Two 39-pin flex connectors, board sits inline (man-in-the-middle):
  most signals pass straight through, capture-relevant signals also
  fan out to STM32 GPIOs.
- Capture mechanism on STM32: WR drives TIM1_ETR; per-edge DMA reads
  of `GPIOA->IDR` and `GPIOB->IDR` into two ring buffers. Data bus,
  DC, and CS are all captured (DC/CS are on PB pins along with the
  data-PB byte). See "Capture mechanism" under MCU above for the
  TIM1 + DMA register details.
- Reference: the Pico 2 W prototype (RP2350 PIO + DMA) validated the
  display-side decode against live 0x2A/0x2B/0x2C + RGB565 traffic
  from an ILI9488/ST7796-compatible controller — see
  [display_notes.md](display_notes.md). The protocol/decode work
  carries over verbatim; only the capture front-end is being re-done
  for the STM32.

### Flex GND tie

The display flex carries three pins suspected to be GND on the
target side (J2 pins 1, 18, 19 — labels `GND_1`, `GND_18`, `GND_19`
in the SKiDL netlist). They are kept as separate nets at the
schematic level because the target flex pinout is partly guessed
and shorting two non-GND pins together would be hard to undo.

Each one is tied to board GND through a **dedicated 0402 0 Ω
jumper** on the top side near the flex connectors. **Populated at
fab time by default** — matches the "prior project validated the
target pinout" baseline and skips a per-board hand-solder step.

Bring-up:
1. Boards arrive with all three 0 Ωs populated.
2. Before plugging into the target, scope each suspected-GND pin
   against board GND while the target is alone-powered, confirming
   it's at 0 V.
3. If any pin shows a non-zero signal, depopulate that 0 Ω with
   hot air.

### Status LED

- One LED on an STM32 GPIO + current-limit resistor (0603 LED + 0603
  ~1 kΩ, both basic-library parts on JLCPCB — adds ~$0.02/board).
- ESP32 status is covered by the Xiao's onboard user LED (GPIO 15), so
  no additional LED needed on the ESP32 side.
- Purpose of the STM32-side LED: independent "STM32 is alive"
  feedback during bring-up — confirms the firmware booted from flash
  and code is running even when the ESP32-side comms path is suspect.

### Bring-up test points

- **General-purpose test points** on free STM32 GPIOs PB3, PB4, PB5
  (contiguous with PB0–PB2 already in use for DC/CS/BOOT1).
- All exposed as small SMD pads at the board edge; no headers needed.

## ESP32-C6 (Xiao) resources

- **Flash:** 4 MB onboard
- **SRAM:** 512 kB
- **GPIO:** ~11 on castellated edges

Flash budget (rough): ~1.5 MB ESP32 app+WiFi, ~1.5 MB OTA mirror,
~64 kB to hold the STM32 firmware image, ~0.5 MB NVS/filesystem.
Fits in 4 MB with OTA; very comfortable now that the STM32 image
is small (was ~50 kB+ for RP2350; STM32 64 kB cap is the upper bound).

### Pin budget

| Function                  | Direction | Notes                              |
|---------------------------|-----------|------------------------------------|
| UART TX → STM32 PA10      | OUT       | Bootloader + runtime, 115200+      |
| UART RX ← STM32 PA9       | IN        | Bootloader + runtime               |
| STM32 NRST                | OD/OUT    | Pulse low to reset                 |
| STM32 BOOT0               | OUT       | High → enter ROM bootloader        |
| PIC32 reset               | OD        | Input=released, low=asserted       |
| Free                      |           | ~5–6 GPIO unallocated              |

## STM32F103C8T6 pin budget

48 pins on LQFP-48; ~37 I/O after subtracting power, ground, NRST,
BOOT0, and the OSC pins (PD0/PD1 on F103, PF0/PF1 on F030 — both
LQFP-48 packages expose them as alternates). Verified against
[`reference/STM32F103/datasheet.pdf`](../reference/STM32F103/datasheet.pdf)
Table 5 (and identical for our pins in the F030 datasheet
[`reference/STM32F030/datasheet.pdf`](../reference/STM32F030/datasheet.pdf)
Table 11).

### Fixed pins (forced by silicon or boot)

| Function                       | Pin    | LQFP-48 # | Constraint        |
|--------------------------------|--------|-----------|-------------------|
| WR (TIM1_ETR)                  | PA12   | 33        | TIM1_ETR has no remap on F103; capture trigger lands here. |
| USART1 RX (boot + runtime)     | PA10   | 31        | ROM bootloader pin (AN2606), no DMA remap. |
| USART1 TX (boot + runtime)     | PA9    | 30        | ROM bootloader pin (AN2606), no DMA remap. |
| SWDIO                          | PA13   | 34        | Debug; firmware AFIO `SWJ_CFG=010` to keep SWD and disable JTAG. |
| SWCLK                          | PA14   | 37        | Same. |
| NRST                           | NRST   | 7         | ESP32 drives open-drain; AN2586 §2.3.3 100 nF to GND for EMS. |
| BOOT0                          | BOOT0  | 44        | ESP32 drives push-pull; 10 kΩ pull-down per AN2586 Fig 10. |
| BOOT1                          | PB2    | 20        | 10 kΩ pull-down to GND; not driven by firmware, never used as I/O. |

### Floating pins (chosen at routing time)

The capture path reads `GPIOA->IDR` and `GPIOB->IDR` as whole 16-bit
ports, so the *logical* DB0–DB15 identity of a pin is whichever
SKiDL net the schematic ties to it — the host decoder applies a
`LOGICAL_TO_PHYSICAL[16]` permutation when framing events.
Within each port, route whatever pin order is convenient.

| Function                       | Allowed pin pool                          |
|--------------------------------|--------------------------------------------|
| Data bus PA-half (8 bits)      | 8 pins chosen from PA0–PA8, PA11, PA15. PA15 needs JTAG-disabled. |
| Data bus PB-half (8 bits)      | 8 pins chosen from PB0–PB15 except PB2 (BOOT1). |
| DC, CS                         | Any 2 free pins on either port — both `GPIOA->IDR` and `GPIOB->IDR` are read every WR edge by parallel DMA channels. |
| Status LED                     | Any free pin outside the boot-strap rails. |

Free pool after a typical 16-bit data + DC/CS + LED allocation:
PA8, PA11, PA15 (JTDI — JTAG disable required), PB3 (JTDO —
likewise), PB4 (NJTRST — likewise), PB6, PB7 (or whichever PB pair
isn't taken by DC/CS), PD0/PD1 (OSC; usable as GPIO when no
crystal is fitted). Plenty of test-point and future-bodge headroom.

## Open questions

These need answers before schematic. Tackle them in roughly this order;
items earlier in the list block later ones.

### target. JLCPCB BOM cost review — every part, not just the flex

Originally just about the flex connector, but JLCPCB shuffles parts
between basic and extended libraries frequently and "extended" adds
both a per-part assembly fee and a reel cost. Worth a sweep over
the whole BOM before submission. Goal: every active part either in
basic, or consciously accepted as extended.

Method: for each part below, look up the current JLCPCB part number
(search the value + footprint on jlcpcb.com/parts), note basic vs
extended, and pick a basic-library substitute where one exists with
a compatible footprint. **Footprint compatibility matters more than
exact part number** — if a cheaper part has the same pad layout
(pitch, lead-out direction, height) we can swap parts without
redesigning.

Current BOM (parts that could move between basic and extended):

| Part / value | Qty | Footprint | Concern |
|---|---|---|---|
| FH26W flex 39p 0.3 mm | 2 | custom | Originally extended-or-missing; check current status and Molex 502598 / JST equivalents |
| STM32F103C8T6 | 1 | LQFP-48 | Basic last we checked; reconfirm |
| 100 nF X7R | 7 | 0402 | Should be basic; verify |
| 1 µF / 4.7 µF | 1+1 | 0402 | Basic for X5R/X7R at 6.3 V+; verify the exact rating |
| 22 µF | 1 | 0805 | Likely basic at low voltage |
| 100 µF | 1 | 1206 | Often extended in low-ESR variants; check basic alternatives or drop to 47 µF |
| 1 kΩ / 10 kΩ resistors | 1+2 | 0402 | Should be basic |
| Status LED | 1 | 0603 | **User-flagged: currently expensive.** Find a basic-library 0603 LED in any colour — colour doesn't matter, footprint does |
| 1×3 pin header 2.54 mm | 2 | through-hole | Through-hole; JLC PCBA only assembles SMD by default — these are hand-soldered, so library status doesn't matter |
| Xiao ESP32-C6 (module) | 1 | castellated | Module is hand-soldered, not assembled by JLC; library status N/A |
| 0 Ω jumpers (Q18, Q19, possibly Q20) | TBD | 0402 | Basic; cheapest possible part — go for whichever is in stock |

Plus the upcoming additions from open questions:
- Q18: 3× 0402 0 Ω for flex GND ties
- Q19: 2× 0402 0 Ω for UART break jumpers
- Q20: 1× 2-pin 2.54 mm header (through-hole, hand-soldered)
- Q21: TBD passives for the PIC32 reset hold-low circuit (R + C + N-FET, all 0402-class if possible)

Output: before BOM submission, fill a column with the JLCPCB part
number + basic/extended status + unit price for every row. If the
total extended-parts surcharge exceeds ~$5–10 per assembly run,
revisit the choices.

### Q6. target 3V3 headroom

Need to confirm there's at least ~400 mA spare on the target 3V3 rail
for Xiao C6 WiFi peaks + STM32 (~30 mA active at 72 MHz) without
browning out the display or the PIC32.

Blocking risk: if there's not enough headroom, fall back to the
5V tap header (J4) and let the Xiao's onboard LDO regenerate 3V3
locally — see "Power: tap target 3V3 rail" → "Fallback".

**Validation procedure** (do in this order, stop when confident):

1. **Identify the target 3V3 regulator.** Find the part on the main
   board, look up its rated output current. If it's rated ≥1 A and the
   target's own 3V3 load is modest (a few hundred mA typical for an
   LCD-driven sensor device), there's likely headroom. If it's a small
   SOT-23 LDO rated ≤300 mA, expect trouble.
2. **Bench supply substitution smoke test.** Power the target from a
   bench supply at its normal input voltage, current-limited to
   (expected target draw + ~400 mA budget for capture PCB). Verify
   normal operation with backlight on. Repeats #1 less precisely but
   tells you whether the *whole device* has total-system headroom even
   if you can't identify the regulator.
3. **Inline shunt measurement** (the definitive answer):
   - Splice a 0.1 Ω shunt into the 3V3 wire of the 3-pin connector.
   - Measure voltage across it with a scope (DMM averages down the
     short WiFi TX peaks; not useful for transient measurement).
   - Take three readings:
     - target alone, idle — baseline.
     - + capture PCB powered, ESP32 idle — quiescent overhead.
     - + ESP32 actively WiFi TX-ing (run a tight `WiFi.begin()` +
       HTTP POST loop) — worst case.
   - Headroom = regulator rated current − (target baseline +
     measured peak).

**Mitigation regardless of result:** add bulk capacitance at the 3V3
input of the capture PCB (22 µF + 100 µF MLCC/tantalum is reasonable)
near the Xiao 3V3 pin, to absorb WiFi TX transients locally rather
than burdening the target regulator's transient response. The Xiao
module has its own onboard decoupling sized for normal USB-powered
operation, but when 3V3 is backfed externally that decoupling is
upstream of our tap point and we shouldn't rely on it alone.

### Q7b. STM32F103 design-guide compliance — RESOLVED

AN2586 in
[`reference/STM32F103/an2586_hardware_development.pdf`](../reference/STM32F103/an2586_hardware_development.pdf)
and RM0008 in
[`reference/STM32F103/rm0008.pdf`](../reference/STM32F103/rm0008.pdf):

- **Reset (NRST):** internal pull-up; AN2586 §2.3.3 recommends a
  pull-down capacitor 10–100 nF to GND for EMS — improves
  protection against parasitic resets. ESP32 drives NRST
  open-drain.
- **BOOT0:** dedicated pin on F103C8 LQFP-48 (pin 44). Pull-down
  to GND with a 10 kΩ resistor (AN2586 Figure 10) so it stays
  low unless the ESP32 actively drives it high. ESP32 drives
  push-pull.
- **PB2 = BOOT1:** F103 has an additional boot-select pin
  (BOOT1) which is muxed with PB2. Latched together with BOOT0
  on the 4th rising SYSCLK edge after reset (AN2586 §4.1). For
  our boot modes (main flash + system memory bootloader) we
  always want **BOOT1=0**. PB2 therefore needs a **10 kΩ
  pull-down to GND** on the board so the boot latch reads 0
  unambiguously even before firmware has configured the pin.
  **PB2 must not be used as a status LED with active-high
  drive**, since that would force BOOT1=1 during reset. Moved
  the status LED to a different pin (see updated pin budget).
- **No external crystal needed**: HSI + PLL → 64 MHz max
  (DS5319 Table 7 note 1). HSI accuracy at 0–70 °C is ±1.3% to
  ±2% (DS5319 Table 24), within UART tolerance at the baud
  rates we use. F103 ROM bootloader auto-bauds on HSI (AN2606).
- **Optional improvement:** if USART1 bootloader timing turns
  out flaky in practice across the temperature range, add an
  8 MHz crystal on OSC_IN/OSC_OUT (PD0/PD1 on F103 LQFP-48) and
  switch HSE + PLL → 72 MHz. Leave a depopulated 3225 crystal
  footprint + 18 pF cap pads as insurance.
- **JTAG vs SWD:** F103 boots with JTAG enabled on PA13–PA15 +
  PB3 + PB4. To free PB3/PB4/PA15 as plain GPIOs while keeping
  SWD on PA13/PA14, firmware must write
  `AFIO_MAPR.SWJ_CFG=010` (JTAG-DP disabled, SW-DP enabled)
  early in startup. None of those pins carry a DB bit in our
  current map, but if we later need PA15/PB3/PB4 as plain GPIO
  inputs/outputs the JTAG-disable step is mandatory.

### Q15. Embassy STM32F1 support — likely fine, verify before firmware port

The current firmware uses `embassy-rp` heavily. STM32F1 support in
Embassy:
- `embassy-stm32` has first-class F1 support (F103C8 is one of the
  most-tested targets in the ecosystem). Status of HAL features we
  depend on:
    - GPIO + DMA: yes.
    - Timer with ETR slave mode: yes, but the high-level API may
      not expose every slave-mode combination — verify by reading
      `embassy-stm32/src/timer/` for `Stm32F103C8` HAL.
    - Two DMA channels driven from the same timer event: not a
      standard high-level API on any STM32 HAL; almost certainly
      needs PAC-level register access. Acceptable.
- The firmware port is out of scope for this spec but the BOM-level
  decisions (which pins go where) need to be made now and feed
  back into the firmware design. If `embassy-stm32` turns out to
  lack some critical primitive, the fallback is writing the
  capture path bare-metal — viable given the small scope of STM32
  firmware here.

**SRAM headroom:** 20 kB on F103 (vs the F030's 8 kB this design
briefly targeted) easily fits an Embassy executor + DMA rings +
USART buffers. This was the main bring-up risk on the F030 draft
and is now de-risked.

### Q17. Host-side bit-permutation table — to populate after routing

The PCB router is free to assign any PA pin to any data-PA bit
and any PB pin to any data-PB bit (subject to the hard constraints
in "Pin allocation" above). The firmware streams raw `GPIOA->IDR`
and `GPIOB->IDR` samples; the host permute layer
([`host/src/permute.rs`](../host/src/permute.rs)) needs
a static table mapping (port, physical bit) → logical DB bit so
that captured samples can be interpreted regardless of routing.

What to produce, in order:

1. Finalise the routing. SKiDL net names (`DB0`..`DB15`, `DC`,
   `CS`) define the logical identity of each pin; KiCad/SKiDL
   pin assignment defines the physical pin.
2. Extract from the netlist a table like:

   ```rust
   const PA_BIT_TO_LOGICAL: [Option<u8>; 16] = [
       Some(0),  // PA0 -> DB0 (example)
       Some(7),  // PA1 -> DB7
       /* ... */
   ];
   const PB_BIT_TO_LOGICAL: [Option<u8>; 16] = [ /* DC, CS, DB-bits */ ];
   ```

3. Decoder applies it once per captured event:

   ```rust
   fn permute(pa: u16, pb: u16) -> (u16 /* dbus */, bool /* dc */, bool /* cs */) {
       let mut dbus = 0u16;
       for (bit, logical) in PA_BIT_TO_LOGICAL.iter().enumerate() {
           if let Some(l) = logical {
               dbus |= ((pa >> bit) & 1) << l;
           }
       }
       // ... same for PB; dc, cs from designated PB bits
       (dbus, dc, cs)
   }
   ```

Cost: ~32 shift/mask ops per event, applied at burst-framing time
(not the per-WR rate). Negligible.

The permutation table should be generated mechanically from the
SKiDL netlist so it can't drift from the schematic — emit it
during the netlist regeneration step. A small Python helper in
`pcb/` can read the netlist and write a Rust source file the
decoder includes via `include!()`.

### Q21. PIC32 reset — move to STM32 + power-on hold-low — TODO

Two related changes to the PIC32 MCLR (currently `PIC32_RESET`, open-
drain from ESP32 GPIO20):

**(a) Move control from ESP32 to STM32.** The STM32 has spare GPIOs
and is the deterministic, low-latency side of the board (no WiFi
stack to compete with). It's also the side that knows when capture
DMA is armed, so it can hold the target in reset until it's actually
ready to record. Route MCLR to a free STM32 GPIO (PB6 or PB7 are
obvious candidates — adjacent, both free, on the same side of the
package as the rest of the PB cluster). Drive it the same way as
today: open-drain, **never drive high**.

**(b) Power-on hold-low.** On board power-on, MCLR should be held
asserted (= low, **double-check polarity against the PIC32 part
on the target main board before fab**) until the STM32 firmware is
running and has explicitly released it. Otherwise the target starts
running while our capture chain is still in reset, and we miss the
boot-time display traffic.

Simple implementation: an RC delay + small N-MOSFET (or open-drain
buffer) pulling MCLR low, driven by the STM32's GPIO once firmware
boots. Or even simpler: rely on the fact that the STM32 GPIO is
high-Z at reset → MCLR pulled high by the target's own pull-up →
target runs. That's the *opposite* of what we want; we want the
default to be "target held in reset until we say go".

Sketches to evaluate:
1. **RC + transistor.** R from 3V3 to GND through a cap; gate of an
   N-FET sees the cap voltage. On power-up the cap is empty → FET
   on → MCLR pulled low. Cap charges through R; FET turns off after
   ~τ. Meanwhile STM32 firmware boots and either takes over with
   its own GPIO pulling MCLR low (keeps it asserted) or releases
   (lets it go high). Tunable: R·C ≥ STM32 boot-to-take-control
   time, with margin.
2. **Diode-OR'd reset lines.** STM32 GPIO and a power-on-reset chip
   (TPS3823 or similar) both pull MCLR open-drain. Either one
   asserted → MCLR low. POR chip handles the cold-boot window;
   STM32 takes over thereafter. Cleaner electrically, more BOM.
3. **Default-asserted-via-STM32-NRST-chain.** Tie MCLR to STM32
   NRST via an open-drain buffer so STM32-in-reset = MCLR low.
   Cheapest, but couples the two reset domains — if the STM32
   crashes and reboots, the target reboots with it.

**Pick sketch #1 (RC + transistor) unless something rules it out** —
it's smallest, has the right default behaviour, and the timing is
robust because the STM32 takes over before the cap finishes
charging (so the exact RC value isn't load-bearing).

⚠ **Polarity double-check** before BOM: PIC32 MCLR is conventionally
active-low (asserted = low). Confirm against the specific PIC32
part on the target main board — prior-project note says "open-drain
with pull-up on the main board" which is consistent with active-low,
but worth re-verifying with a scope during bring-up.

ESP32-side impact: GPIO20 is freed up by this move and becomes a
spare GPIO on the Xiao header. Update the pin budget table once the
move is committed.

## References

Primary (current MCU — F103C8T6):
- STM32F103x8/xB datasheet (DS5319) —
  [`reference/STM32F103/datasheet.pdf`](../reference/STM32F103/datasheet.pdf)
- STM32F10xxx RM0008 reference manual —
  [`reference/STM32F103/rm0008.pdf`](../reference/STM32F103/rm0008.pdf)
- AN2586 — Getting started with STM32F10xxx hardware development —
  [`reference/STM32F103/an2586_hardware_development.pdf`](../reference/STM32F103/an2586_hardware_development.pdf)

Common (apply to both F030 and F103):
- AN2606 — STM32 microcontroller system memory boot mode (USART1
  bootloader on PA9/PA10) —
  [`reference/an2606_bootloader.pdf`](../reference/an2606_bootloader.pdf)

Secondary (F030C8T6 alternative, kept for reasoning context):
- STM32F030C8 datasheet —
  [`reference/STM32F030/datasheet.pdf`](../reference/STM32F030/datasheet.pdf)
- STM32F0 RM0360 reference manual —
  [`reference/STM32F030/rm0360.pdf`](../reference/STM32F030/rm0360.pdf)

Project artefacts:
- [display_notes.md](display_notes.md) — captured protocol details
- RP2350 prototype firmware (PIO+DMA capture front-end):
  [`firmware/src/pio_capture.rs`](../firmware/src/pio_capture.rs).
  Bus-protocol decode lives host-side now in
  [`host/src/bus_decoder.rs`](../host/src/bus_decoder.rs).
