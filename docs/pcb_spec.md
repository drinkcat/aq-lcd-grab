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

##### Pin map (proposal)

Capture splits across two ports, reading low byte from PA and high
byte from PB, with control bits on remaining PB pins. USART1 stays
on PA9/PA10 permanently — it's the single ESP32 ↔ STM32 link for
both ROM-bootloader flashing and runtime data, so it cannot be
multiplexed with data-bus capture.

| Signal       | Pin   | LQFP-48 # | AF / role             |
|--------------|-------|-----------|------------------------|
| DB0–DB7      | PA0–PA7 | 10–17   | GPIO input (contiguous edge) |
| DB8–DB15     | PB8–PB15 | 45,46,21,22,25,26,27,28 | GPIO input (scattered, see Q13) |
| WR (sample clock) | PA12  | 33      | TIM1_ETR (no AF mapping needed on F103 — TIM1_ETR is the default function of PA12) |
| D/C          | PB0   | 18        | GPIO input             |
| CS           | PB1   | 19        | GPIO input             |
| BOOT1 latch  | PB2   | 20        | **10 kΩ pull-down to GND**, not driven |
| Status LED   | PB5   | 41        | GPIO output (active either polarity — pin is free of boot-strap meaning) |
| USART1 RX    | PA10  | 31        | USART1_RX (bootloader + runtime) |
| USART1 TX    | PA9   | 30        | USART1_TX (bootloader + runtime) |
| SWDIO/SWCLK  | PA13/PA14 | 34/37 | Debug                  |
| NRST         | NRST  | 7         | ESP32 reset            |
| BOOT0        | BOOT0 | 44        | ESP32 BOOT0            |

Full 16-bit data bus is captured: PA9/PA10 carry no data-bus
signal, but they didn't need to — DB8–DB15 fits cleanly in 8 PB
pins. PB2 cannot be repurposed because it's the BOOT1 latch; on
F030 (briefly considered alternative) PB2 had no boot-strap
meaning, which is why the earlier draft used it as the LED.

##### Capture mechanism (2–3 DMA channels)

TIM1 in **external clock mode 2** (ECE=1 in TIM1_SMCR), clocked by
ETR (= WR). With **ARR=0**, every ETR edge increments the counter
to 0 and triggers an update event (UEV); CC1/CC2 channels in
output-compare mode with CCR=0 generate compare-match events on
the same cycle.

Each TIM1 event has an independent DMA request line, enabled via
the DIER register: UDE, CC1DE, CC2DE, etc. (RM0360 §13.4.4). Each
line maps to a fixed DMA channel (RM0360 Table 26):

| TIM1 source | DMA channel | Notes                            |
|-------------|-------------|----------------------------------|
| TIM1_UP     | Ch 5        | reads `GPIOA->IDR` → data-low ring |
| TIM1_CH1    | Ch 2        | reads `GPIOB->IDR` → data-high+ctrl ring |
| TIM1_CH2    | Ch 3        | optional: counter snapshot ring  |

All three fire on the same WR edge because they're triggered by
different TIM1 events that all occur at the overflow point.

**Channel-2 conflict check:** Ch 2 is also the default mapping for
`USART1_TX` DMA. We handle this by running USART1 TX in
interrupt-driven (non-DMA) mode — Embassy supports both, and at the
ESP32 link's baud rate (115200–1 Mbaud) the CPU can sustain the
ISR load easily. Alternatively, set the USART1_TX remap bit in
SYSCFG_CFGR1 to relocate it to Ch 4, leaving Ch 2 untouched. (See
RM0360 §6.4.3 USART1_TX_DMA_RMP.)

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
  command/data framing line (see [decoder.rs](../firmware/src/decoder.rs)).
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
- **Ring buffer sizing:** average burst is ~3,300 words × 2 bytes
  (data) + ~3,300 × 2 bytes (control) ≈ 13 kB total. **8 kB SRAM
  is the hard limit** and we also need stack + USART buffers + code
  state. Strategies (TBD in firmware bring-up):
    1. Stream out over USART1 to the ESP32 mid-burst rather than
       buffering the whole burst — possible because USART1 at
       e.g. 1 Mbaud can sustain ~100 kB/s vs the ~13 kB/s average
       data rate. Risk: instantaneous burst rate is ~2.6 MB/s; UART
       must absorb the differential via a smaller ring.
    2. Compress in firmware (RLE on data, like the RP2350 firmware
       does for uniform pixel fills) before forwarding.
    3. Pack control bits — DC and CS are 1 bit each, so a byte per
       sample halves the control-ring memory. Even better: pack DC
       into the unused MSB of the 16-bit data sample, since `GPIOA
       ->IDR` upper bits are zero and we have a free bit. Then DMA
       reads 16-bit `GPIOA->IDR | (GPIOB << shift)` — but the F030
       doesn't have hardware bit-merge between ports, so this would
       need a CPU-side combine, defeating DMA. Stick with separate
       rings.
- **Risk to validate in firmware bring-up:** the F030's DMA must
  sustain 667 kHz × 2 channels of GPIO→SRAM transfers without
  dropping samples during the burst. At 48 MHz that's one transfer
  per ~36 CPU cycles across both channels. AHB-Lite is single-master
  but DMA bursts a single 16-bit read+write in a few cycles, so
  this should fit — but it's the new front-end risk, replacing
  RP2350 PIO. The Pico 2 W prototype already required care
  (filtering glitches, ring-mode DMA); the F030 path needs
  equivalent attention.

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

#### USB recovery — removed

The F103C8T6 *does* have USB device (the F030C8T6 alternative did
not). For this design we still skip USB on the board:

- The intended runtime data path is ESP32 ↔ STM32 USART, not host
  USB. The capture board sits inside the target, not on a desk
  next to a laptop.
- The SWD + UART-bootloader recovery paths already cover failure
  modes. Adding USB would add a connector, ESD diodes, two ~22 Ω
  series resistors, board area, and complicate the chassis
  cut-out — for marginal benefit.
- Could be revisited if a future bring-up campaign would benefit
  from high-bandwidth direct host streaming; the F103's USB
  peripheral leaves that door open without affecting the current
  BOM.

Net effect vs the RP2350 plan:
- Drop USB-C connector + 27 Ω resistors + 0 Ω VBUS jumper.
- Drop the 4-pin USB-C header.
- Add a 4-pin SWD header (SWDIO/SWCLK/GND/3V3) instead.

### Power: tap target 3V3 rail

- Single 3V3 rail powers the whole capture board (RP2350, Xiao C6,
  level translation if any).
- Backfed into Xiao C6 3V3 pin, which bypasses its onboard LDO. This
  means the Xiao cannot be powered from its own USB-C while the
  capture PCB is connected to the target.
- **To reflash the ESP32 standalone:** disconnect the 3V3 pin in the
  3-wire connector between the capture PCB and the target main board.
  This isolates both the ESP32 and the RP2350 from the target supply,
  letting the Xiao run from its own USB-C.
- Caveat: if USB on the RP2350 is also plugged in with VBUS connected
  (see Q2), don't have both USB cables live at once unless an OR'ing
  scheme is built — they'd fight on the 3V3 rail.
- **Risk to validate:** Xiao C6 WiFi TX pulls 200–300 mA peaks. target
  3V3 must source that on top of the display backlight load. Measure
  the target 3V3 headroom before committing.

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
- Capture mechanism on STM32: WR drives TIM1_ETR; TIM1 update event
  triggers DMA from `GPIOA->IDR` (containing DB0–DB15 on PA0–PA15)
  into an SRAM ring buffer. DC/CS captured separately (PB1, PB2)
  either via a second DMA channel or by sampling alongside in a
  16-bit-wide DMA transfer of a remapped GPIO port. To be finalised
  during firmware bring-up.
- Reference: the Pico 2 W prototype (RP2350 PIO + DMA) validated the
  display-side decode against live 0x2A/0x2B/0x2C + RGB565 traffic
  from an ILI9488/ST7796-compatible controller — see
  [display_notes.md](display_notes.md). The protocol/decode work
  carries over verbatim; only the capture front-end is being re-done
  for the STM32.

### Status LED

- One LED on an STM32 GPIO + current-limit resistor (0603 LED + 0603
  ~1 kΩ, both basic-library parts on JLCPCB — adds ~$0.02/board).
- ESP32 status is covered by the Xiao's onboard user LED (GPIO 15), so
  no additional LED needed on the ESP32 side.
- Purpose of the STM32-side LED: independent "STM32 is alive"
  feedback during bring-up — confirms the firmware booted from flash
  and code is running even when the ESP32-side comms path is suspect.

### Bring-up test points

- **2× hardware USART2 test points** on STM32 **PA2 (TX) / PA3 (RX)**
  — secondary serial for ESP32 ↔ STM32 comms bodging if the primary
  USART1 path (PA9/PA10, shared with the ROM bootloader) turns out
  to have an issue, or for an attached serial console during bring-up.
- **Additional general-purpose test points** on free STM32 GPIOs (TBD
  in layout — likely PB3, PB4, PB5 since they're contiguous with PB0–
  PB2 already in use for WR/DC/CS).
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

| Function                       | Pin    | LQFP-48 # | AF / role        |
|--------------------------------|--------|-----------|------------------|
| DB0–DB7                        | PA0–PA7 | 10–17    | GPIO input       |
| DB8 / DB9                      | PB8 / PB9 | 45 / 46 | GPIO input       |
| DB10 / DB11                    | PB10 / PB11 | 21 / 22 | GPIO input    |
| DB12 / DB13                    | PB12 / PB13 | 25 / 26 | GPIO input    |
| DB14 / DB15                    | PB14 / PB15 | 27 / 28 | GPIO input    |
| WR                             | PA12   | 33        | TIM1_ETR         |
| D/C                            | PB0    | 18        | GPIO input       |
| CS                             | PB1    | 19        | GPIO input       |
| BOOT1                          | PB2    | 20        | **10 kΩ pull-down to GND** (not driven by firmware) |
| Status LED                     | PB5    | 41        | GPIO output      |
| USART1 RX (boot + runtime)     | PA10   | 31        | USART1_RX (AF default) |
| USART1 TX (boot + runtime)     | PA9    | 30        | USART1_TX (AF default) |
| SWDIO                          | PA13   | 34        | Debug (post-AFIO `SWJ_CFG=010`) |
| SWCLK                          | PA14   | 37        | Debug (post-AFIO `SWJ_CFG=010`) |
| NRST                           | NRST   | 7         | ESP32 drives OD; AN2586 100 nF to GND for EMS |
| BOOT0                          | BOOT0  | 44        | ESP32 drives PP; 10 kΩ pull-down per AN2586 |
| Free                           | PA8, PA11, PA15 (JTDI – needs JTAG disabled), PB3 (JTDO – needs JTAG disabled), PB4 (NJTRST – needs JTAG disabled), PB6, PB7, PD0/PD1 (OSC, can be GPIO if no crystal) | — | — |

**Pin contiguity matters but is broken by the package.** DB0–DB7 sit
on a clean physical edge (LQFP-48 pins 10–17). DB8–DB15 scatter
across pins 21–28 + 45–46 on the opposite half of the package — the
display flex connector must be positioned so its DB8–DB15 traces can
wrap to those pins (Q13).

**USART1 is non-negotiable on PA9/PA10.** It carries both the ROM
bootloader handshake (for ESP32-driven reflashing) and the runtime
ESP32 ↔ STM32 link. There is no plan to share these pins with any
data-bus bit — earlier draft proposed PA9/PA10 as multifunction
DB9/DB10 + USART1, which would have made runtime UART chatter
collide with display traffic. Permanent UART is required.

## Open questions

These need answers before schematic. Tackle them in roughly this order;
items earlier in the list block later ones.

### Q3. STM32 GPIO assignment for display capture — partial

Datasheet Tables 11/12/13 are now in
[`reference/STM32F030/datasheet.pdf`](../reference/STM32F030/datasheet.pdf)
and have been used to draft the pin map in the "Pin map (proposal)"
section above. Key realities:

- PA12 = TIM1_ETR (only ETR pin on F030C8 — TIM3_ETR is PD2,
  unbonded on LQFP-48). WR must land here for direct ETR trigger.
- PA13/PA14 = SWDIO/SWCLK by reset; keep them for debug.
- PA9/PA10 = USART1, **permanent**: ROM bootloader + runtime
  ESP32 ↔ STM32 link. Cannot be repurposed as data-bus bits — at
  runtime any UART character would collide with display traffic.
- DB0–DB7 fit naturally on PA0–PA7 (LQFP-48 pins 10–17, one edge).
- DB8–DB15 placed on PB8–PB15 (8 pins, no DB bits sacrificed —
  USART1 doesn't need any PB pin since the default PA9/PA10 mux is
  used).
- DC, CS on PB0, PB1; status LED on PB2.

Resolved-but-with-followups:
- DB8–DB15 on PB8–PB15 implies the high byte of `GPIOB->IDR` is
  read by a second DMA channel — verify F030's DMA can carry two
  TIM-triggered transfers in parallel (Q12).
- Physical layout of PB8–PB15 is scattered across the LQFP-48
  (pins 21, 22, 25–28, 45, 46) — see Q13.

### Q9. Ground pour underneath the flex pass-through?

The bottom-side GND pour is interrupted directly under the flex
pass-through traces between J1 and J2 if we route them on the top layer
without via-stitching across. Question:

- Does the 16-bit data bus + 8080 control signals need a continuous
  GND reference plane underneath for signal integrity at the (modest)
  display write-strobe speed?
- Or is it fine to break the pour, since the parallel display protocol
  is single-ended and slow?

Probably fine to break the pour given the slow signals, but worth
checking once we have a real placement to see how much pour we'd
actually be cutting.

### target. JLCPCB-friendly flex connector sourcing

Currently using `FH26W:FH26W39S03SHW60` (Hirose FH26W, 39-pin 0.3 mm
pitch). Need to check:

- Is this part in JLCPCB's basic library? If not, what's the
  extended-library availability and price?
- Are there cheaper drop-in alternatives in JLCPCB basic that take
  the same flex (39-pin, 0.3 mm pitch, dual contact)? Common
  alternatives: Molex 502598-3990 / 502598 series, JST 39FMN-BMT,
  generic AliExpress equivalents.
- Footprint compatibility matters more than exact part number — if a
  cheaper part has the same pad layout (same pitch, same lead-out
  direction, same height), we can swap parts without redesigning.

### Q11. Inductor polarity — OBSOLETE

Was about the RP2350 SMPS inductor (AOTA-B201610S3R3-101-T). The
STM32F030 doesn't have an internal SMPS, so there's no external
inductor to polarise. Question dropped.

### Q6. target 3V3 headroom

Need to confirm there's at least ~400 mA spare on the target 3V3 rail
for Xiao C6 WiFi peaks + STM32 (~30 mA active at 48 MHz, much less
than the RP2350's ~50 mA) without browning out the display or the
PIC32.

Blocking risk: if there's not enough headroom, need a separate power
input (USB barrel jack, or 5V tap if available on the main board).

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

### Q7. Decoupling and reference layout

Two distinct concerns — don't conflate them:

**(a) Bulk caps for WiFi TX transients** — covered in Q6's mitigation
above. 22 µF + 100 µF near the Xiao 3V3 pin.

**(b) STM32 local decoupling** — needed independently. F103
recommended decoupling per AN2586 §2.2
([`reference/STM32F103/an2586_hardware_development.pdf`](../reference/STM32F103/an2586_hardware_development.pdf)):

| Rail              | Caps                                          |
|-------------------|-----------------------------------------------|
| VDD (×2 on LQFP-48) | 100 nF per pin + 1× 4.7–10 µF bulk          |
| VDDA              | 100 nF + 1 µF                                 |
| VBAT (if no battery) | tied to VDD via 100 nF                     |

VREF+ is **not bonded out** on the F103C8 LQFP-48 package
(DS5319 Figure 8 / Table 5 — VREF+ only on LFBGA100 / TFBGA64);
no external decoupling needed.

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

### Q8. Mechanical / form factor

- **Target board size: as small as possible**, ideally close to the
  Xiao C6 footprint (21 × 17.5 mm). The Xiao is mounted on the back
  side of the PCB to leave the front clear for the STM32 and flex
  connectors.
- **Realistic minimum with LQFP-48 STM32: ~25 × 27 mm.** The STM32
  body is 7 × 7 mm and the LQFP-48 footprint with leads is ~9 × 9 mm,
  larger than the 7 × 7 mm QFN-60 footprint of the RP2350. The flex
  connectors are 0.3 mm pitch, 39-pin (Hirose FH52 / Molex 502598
  class), ~14–15 mm × 3–4 mm each. Net: ~3 mm wider and ~2 mm
  taller than the RP2350 design.
- **Layer stack: 4 layers preferred** (sig / GND / 3V3 / sig).
  Reasoning unchanged from the RP2350 version: 39 flex signals per
  connector, ~16+3 tap to STM32, GND reference plane simplifies
  return paths. JLCPCB 4-layer is ~$5 for 5 boards vs ~$2 for
  2-layer.

  **2-layer is feasible** (no USB to worry about now, which slightly
  loosens the constraint) with the same discipline as before:
    - Bottom layer near-continuous GND pour, no signal routing under
      the STM32 or in the flex fanout area.
    - All decoupling caps on bottom layer, short via to chip pin.
    - Two flex connectors arranged so most pass-through traces run
      straight across the top without bottom-layer crossings.
- **Open:** mounting holes — does the board need to fit a specific
  spot inside the target enclosure, or is it free-floating with the
  flex cables and target 3-pin connector doing the mechanical work?
- **Open:** flex connector orientation (top vs bottom entry) — affects
  how the board sits relative to the display and main board, and
  whether the flex cables fold or run straight.

### Q12. DMA channel mapping for parallel-bus capture — RESOLVED

**Architectural plan** (works on both F030 and F103, originally
verified against RM0360 §10.3.2 / Table 26 / §13.3.7 / §13.4.4
during the F030 draft):

- TIM1 in external clock mode 2 (ECE=1), ETR = WR, ARR = 0.
- Each ETR edge generates UEV + CC1 match (+ CC3 match on F103,
  see below) simultaneously.
- DIER enables UDE, CC1DE, CC3DE independently — multiple DMA
  requests fired per WR edge.
- ETR has an in-built digital filter (ETF[3:0]) replacing the
  RP2350 PIO's software glitch-reconfirm. Set ETF to 2–4 samples
  at fDTS to reject ringing on the falling edge.

**F103 DMA channel mapping** verified against RM0008 Table 78
([`reference/STM32F103/rm0008.pdf`](../reference/STM32F103/rm0008.pdf)):

| TIM1 event  | DMA1 channel | Notes                              |
|-------------|--------------|-------------------------------------|
| TIM1_CH1    | Ch 2         | reads `GPIOA->IDR` → data-low ring  |
| TIM1_UP     | Ch 5         | reads `GPIOB->IDR` → data-high+ctrl ring |
| TIM1_CH3    | Ch 6         | optional: counter snapshot ring     |
| TIM1_CH4 / TRIG / COM | Ch 4 | (unused)                       |

**Important: TIM1_CH2 has no DMA request on F103.** This differs
from F030 (where TIM1_CH2 → Ch3). The architectural plan therefore
uses TIM1_CH1 + TIM1_UP as the two capture-channel triggers
(verified F103-style), and TIM1_CH3 as the optional third.

**Channel-5 USART1 conflict:** Ch5 is also the default DMA target
for `USART1_RX`. Resolution: run USART1 RX interrupt-driven (no
DMA) — at the ESP32 link rates (115200 to 1 Mbaud) the CPU keeps
up trivially. F103 has an AFIO USART1 pin remap (PA9/PA10 →
PB6/PB7) but **not** a DMA remap, so DMA conflicts can only be
resolved at the peripheral usage level.

**Firmware impact:** the capture front-end is register-level
configuration; Embassy's high-level timer API likely won't expose
every knob, so plan on PAC-level setup for TIM1 and DMA. The
application code is small (one task to drain the ring, one to
flush bursts to USART1).

### Q13. Flex-fanout layout for PB8–PB15 scatter — needs PCB sketch

The DB8–DB15 bits land on PB8–PB15. On LQFP-48 these pins are at
package positions 21, 22, 25, 26, 27, 28, 45, 46 — i.e. spread
across two opposite sides of the chip (pins 21–28 on side 3,
pins 45–46 on side 4 near BOOT0). DB0–DB7 on PA0–PA7 are nicely
clustered on pins 10–17 (side 2).

This means the display flex connector ideally sits on the side of
the board nearest pins 10–28 (the long stretch covering PA0–PA7
+ PB0–PB2 + PB10–PB15), and DB14/DB15 (PB8/PB9 at pins 45/46) have
the longest fanout.

Open questions:
- Is the fanout routable on 2 layers, given the GND-pour-under-flex
  concern in Q9? On 4 layers it's straightforward.
- Are there free shorter-path PB pins we should swap into the
  DB14/DB15 slots to shorten the routes? **Software is indifferent
  to physical pin order** — DB14/DB15 are just whichever PB-bits
  the firmware decides to call "bit 14" and "bit 15" — but the
  *logical bit position* matters because we need a contiguous
  high-byte read. The least-painful swap is to choose which two
  PB pins are easiest to route and assign them to DB14/DB15.

### Q14. Bring-up serial conflicts with the data bus

USART2 alternates land on PA2/PA3 (AF1). PA2/PA3 are also DB2/DB3
in the proposed pin map. So enabling USART2 as a console requires
giving up two data bits.

Options:
- Skip USART2 entirely; use SWO (SWD trace) for printf-style debug
  via the SWD header. ST-Link supports this.
- Provide a bring-up jumper that disconnects DB2/DB3 from the flex
  and patches them to the USART2 pad cluster for debug builds.
  Test-only configuration; production builds keep them as data bits.
- Tap USART2 onto PB-port pins (none of the F030 USART2 remaps go
  to PB on the F030 — verify against AF tables; if no PB remap
  exists, skip USART2).

Lean toward "SWO via SWD probe" — no extra pin cost, no fab-time
options.

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

### Q16. RM0360 — RESOLVED

Downloaded by user to
[`reference/STM32F030/rm0360.pdf`](../reference/STM32F030/rm0360.pdf).
Key sections consulted for Q12: §10.3.2 (DMA request routing),
Table 26 (channel/peripheral matrix), §13.3.7 + Figure 69 (TIM1
external clock mode 2 / ETR filter), §13.4.4 (DIER bits CC1DE,
CC2DE, UDE).

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
- RP2350 prototype firmware (display-protocol decode that carries
  over): [`firmware/src/pio_capture.rs`](../firmware/src/pio_capture.rs),
  [`firmware/src/decoder.rs`](../firmware/src/decoder.rs)
