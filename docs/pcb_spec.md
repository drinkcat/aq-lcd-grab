# target device Capture PCB — Design Spec

Working spec for an inline capture board: sits between the target device
main board and its LCD via two 39-pin flex connectors, taps the
display bus into an RP2350 for capture, and hosts a Xiao ESP32-C6 for
WiFi-side processing and as the RP2350's program loader.

Status: draft. Decisions captured below; open questions at the bottom.

## Overview

```
                  +-------------------+
   target main    |    Capture PCB    |     target LCD
   board          |                   |
        flex 39p  |  RP2350 (no flash)|  flex 39p
   ============== | passthrough + tap |==============
                  |                   |
                  |  Xiao ESP32-C6    |
                  |  (UART loader +   |
                  |   WiFi)           |
                  +-------------------+
                            |
                       3-pin to main
                       board (3V3/GND/
                       PIC32 reset)
```

## Decisions

### MCU: RP2350 in flashless UART-boot mode

- **No QSPI flash, no OTP burns.** The RP2350 bootrom falls through to
  UART boot by default on a blank device with QSPI SD1 strapped high
  (datasheet §5.2.2 Table 451, §5.8). On every reset the bootrom waits
  for the ESP32 to push an SRAM image over UART at 1 Mbaud.
- **Strap pins (hard-wired, no GPIO needed):**
  - `QSPI_CSn` → GND (or pull-down) — selects BOOTSEL mode
  - `QSPI_SD1` → 3V3 via pull-up — selects UART boot inside BOOTSEL
  - `QSPI_SD2` → ESP32 UART RX (RP2350 TX, fixed 1 Mbaud, 8N1, LSB first)
  - `QSPI_SD3` → ESP32 UART TX (RP2350 RX)
- **12 MHz crystal** on XIN/XOUT. Bootrom default assumes 12 MHz; no
  OTP config needed.
- **520 kB SRAM** available for the loaded image. Image must be linked
  to `0x20000000` and contain a valid IMAGE_DEF block (Pico SDK
  generates this).
- **Load time:** at 1 Mbaud, each `w` command transfers 32 payload
  bytes in ~3.4 ms (33 bytes out + 1 echo). Current firmware is ~50 kB
  loadable (text+data), so a full reload takes ~5 s. A worst-case full
  520 kB image would take ~57 s. The 1 Mbaud cap is bootrom-fixed; the
  loaded firmware can then use any baud rate it wants for runtime
  comms with the ESP32.
- **Soft reboot behavior:** after `'x'` execute command, bootrom
  reboots and finds the RAM image. With SD1 still strapped high, any
  subsequent reset/crash lands back in UART boot — ESP32 reloads. This
  is the desired behavior (no flash means no persistence between power
  cycles; ESP32 is the source of truth for firmware).
- **Runtime UART after boot:** SD2 and SD3 mux to **hardware UART0
  (F11 alternate function)** — see datasheet GPIO bank 1 function
  table. So once the bootrom hands off, the same two wires become a
  stock hardware UART. No PIO state machines consumed, no extra pins
  routed, any runtime baud rate. The bootrom uses the same pins via
  QMI for its 1 Mbaud bit-banged UART boot, then firmware re-muxes to
  F11 for the PL011 hardware UART.

#### Tradeoff: flashless UART boot vs alternatives

This adds **~6–7 s of boot latency** every power-on (1–2 s ESP32
self-init + ~5 s UART transfer of the current ~50 kB image, growing
linearly with firmware size). Considered alternatives:

| Option | Cost delta | Boot time | Update path | Verdict |
|---|---|---|---|---|
| **RP2350 + UART boot (chosen)** | — | ~6–7 s | OTA on ESP32 — ESP32 holds canonical firmware, no flash programming needed | ✓ |
| RP2354B (RP2350 + 2 MB in-package flash) | +~$0.80/board, QFN-80 (10×10 mm vs 7×7 mm) — worse for the small-board goal | ~100 ms | Need an ESP32-side flash programmer (SWD bit-bang, or UART-boot a flashing stub then write internal flash). C6 has no USB host so picotool-over-USB isn't an option. | Not worth it for our reboot pattern |
| RP2350 + external SPI flash + stub | +~$0.20 flash + a chip, same QFN-60 | ~100 ms | Same complication as RP2354B (need flash programmer on ESP32 side) | Same downsides without the package penalty, but still adds the flash-update mechanism |

**Why UART boot wins for this device:** it's plugged in and runs
continuously; reboots are rare. The 6 s cost is paid almost never,
and in exchange the firmware-update path is trivially "OTA the ESP32"
with no flash-programming protocol to design. The ESP32 being the
single source of truth for firmware also means no risk of RP2350/ESP32
firmware-version skew.

### USB recovery access (D+/D-)

Added as insurance against the unlikely case the UART boot path
breaks (firmware bug in the ESP32-side loader, signal integrity issue,
etc.). USB-C is on the RP2350's dedicated USB pins (not muxed with
QSPI), so it doesn't interfere with UART boot strapping.

- 27Ω series resistors on D+/D- near the chip (per Pico 2 reference)
- 90Ω differential pair routing, short, no stubs
- **Connector: 4-pin 2.54 mm header** (VBUS / D+ / D− / GND, pinout
  documented on silkscreen). Larger than ideal but cheap, lets a USB
  pigtail be soldered or crimped on when needed.
- **VBUS wired to ESP32 5V input** so the same header can double as a
  5V tap point if we later need to tap the target's 5V supply (in case
  3V3 headroom turns out insufficient — see Q6). Join via a **0Ω
  jumper** in a SOD-123 footprint — fine because we control both ends
  and won't plug USB and a 5V tap in simultaneously. The footprint
  takes an SS14 Schottky later if backfeed isolation ever becomes
  needed.

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
  fan out to RP2350 GPIOs.
- Reuses the working Pico 2 W capture design (PIO-based, validated
  against live 0x2A/0x2B/0x2C + RGB565 traffic from an ILI9488/ST7796-
  compatible controller — see [display_notes.md](display_notes.md)).

### Status LED

- One LED on an RP2350 GPIO + current-limit resistor (0603 LED + 0603
  ~1 kΩ, both basic-library parts on JLCPCB — adds ~$0.02/board).
- ESP32 status is covered by the Xiao's onboard user LED (GPIO 15), so
  no additional LED needed on the ESP32 side.
- Purpose of the RP2350-side LED: independent "RP2350 is alive"
  feedback during bring-up — confirms UART boot succeeded and code is
  running even when the ESP32-side comms path is suspect.

### Bring-up test points

- **2× hardware UART1 test points** on RP2350 **GPIO 20 (UART1 TX)**
  and **GPIO 21 (UART1 RX)** — F2 alternate function on those pins,
  PL011 hardware UART. For ESP32 ↔ RP2350 comms bodging if the
  SD2/SD3 runtime UART path turns out to have an issue, or for an
  attached serial console during bring-up.
- **3× additional general-purpose test points** on free RP2350 GPIOs
  **19, 22, 23** — for any future bodging need (extra signals,
  PIO-driven secondary UART, debug probe, etc.). Contiguous with the
  UART pair for a tidy 5-pad cluster (GPIO 19, 20, 21, 22, 23).
- All 5 exposed as small SMD pads or castellated pads at the board
  edge; no headers needed.

## ESP32-C6 (Xiao) resources

- **Flash:** 4 MB onboard
- **SRAM:** 512 kB
- **GPIO:** ~11 on castellated edges

Flash budget (rough): ~1.5 MB ESP32 app+WiFi, ~1.5 MB OTA mirror,
~0.5 MB to hold the RP2350 image, ~0.5 MB NVS/filesystem. Fits in
4 MB with OTA; comfortable without it.

### Pin budget

| Function                  | Direction | Notes                        |
|---------------------------|-----------|------------------------------|
| UART TX → RP2350 SD3      | OUT       | 1 Mbaud                      |
| UART RX ← RP2350 SD2      | IN        | 1 Mbaud                      |
| RP2350 RUN                | OUT       | Push-pull OK (RUN has int. pull-up) |
| PIC32 reset               | OD        | Input=released, low=asserted |
| Status LED (TBD)          | OUT       | See open Q5                  |
| Free                      |           | ~6–7 GPIO unallocated        |

## RP2350 pin budget (sketch)

Roughly 30 GPIO on QFN-60. Allocation:

| Function                      | Pins  |
|-------------------------------|-------|
| QSPI straps (CSn, SD1)        | 2     |
| QSPI as UART boot (SD2, SD3)  | 2     |
| 12 MHz crystal (XIN/XOUT)     | 2     |
| USB D+/D- (dedicated)         | 2     |
| Display capture (data + ctrl) | ~18   |
| Free                          | rest  |

Display signal pin assignment, from the Pico 2 W prototype firmware
(see [pio_capture.rs](../firmware/src/pio_capture.rs)):

| GPIO  | Signal       | Notes                                                |
|-------|--------------|------------------------------------------------------|
| 0–15  | DB0–DB15     | 16-bit data bus, LSB first                           |
| 16    | "D/C" (best guess) | Used as the 8080 cmd/data framing line          |
| 17    | "CS"  (best guess) | Other 8080 control line; captured but not framed |
| 18    | WR           | Write strobe, sample trigger                         |

The PIO program (`in pins, N`) requires **consecutive GPIOs starting
from a base pin** — so the contiguous GPIO 0–18 allocation is
load-bearing and must be preserved on the new PCB. On the QFN-60,
GPIOs 0–18 are clustered along one side of the package, which fits
naturally with the display flex connector on that edge.

## Open questions

These need answers before schematic. Tackle them in roughly this order;
items earlier in the list block later ones.

### Q3. RP2350 GPIO assignment for display capture

The Pico 2 W prototype used specific GPIOs for the PIO capture program.
Two choices:

- **Keep the same GPIO numbers** as the Pico 2 W prototype → PIO program
  unchanged, less re-validation work, but routing on the PCB may be
  awkward depending on where the flex connectors land.
- **Reassign for routing convenience** → cleaner layout, but the PIO
  program needs updating (pin base offsets) and re-testing.

Need to enumerate the prototype's GPIO assignment and overlay it on the
proposed board layout before deciding.

### Q6. target 3V3 headroom

Need to confirm there's at least ~400 mA spare on the target 3V3 rail
for Xiao C6 WiFi peaks + RP2350 (~50 mA active) without browning out
the display or the PIC32.

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

**(b) RP2350 local decoupling** — needed independently. Copy the Pico
2 reference schematic exactly:

| Rail              | Caps                              |
|-------------------|-----------------------------------|
| VDD (digital, ×4) | 100 nF per pin                    |
| AVDD              | 100 nF + 10 µF + ferrite bead     |
| DVDD (core, ×2)   | 100 nF per pin + 1 µF bulk        |
| VREG_VIN          | 1 µF                              |
| VREG_VOUT         | 2.2 µF                            |
| USB_VDD           | 100 nF                            |
| QSPI_IOVDD        | 100 nF                            |

Total roughly 10× 100 nF + a handful of larger caps. Not really an
open question, just a checklist item for schematic capture.

### Q8. Mechanical / form factor

- **Target board size: as small as possible**, ideally close to the
  Xiao C6 footprint (21 × 17.5 mm). The Xiao is mounted on the back
  side of the PCB to leave the front clear for the RP2350 and flex
  connectors.
- **Realistic minimum: ~22 × 25 mm.** Flex connectors are 0.3 mm
  pitch, 39-pin (Hirose FH52 / Molex 502598 class), ~14–15 mm × 3–4 mm
  each. With both connectors on opposite edges of the same face, the
  board needs to be at least ~22 mm wide (RP2350 + decoupling +
  fanout) by ~25 mm long (two flex connectors + spacing).
- **Layer stack: prefer 4 layers** (sig / GND / 3V3 / sig). The 0.3 mm
  pitch flex fanout has 39 signals per connector — most pass through,
  ~18 tap to RP2350 — and a real GND plane underneath simplifies the
  return paths for the display bus and USB. JLCPCB 4-layer is ~$5
  for 5 boards vs ~$2 for 2-layer; the extra $3 buys a clean reference
  plane and dedicated power plane, which makes the layout much easier.

  **2-layer is feasible if needed**, but only with discipline:
    - Bottom layer is a near-continuous GND pour, no signal routing
      under the RP2350 or in the flex fanout area.
    - All decoupling caps on bottom layer, GND pad straight to the
      pour, VDD via'd up to the chip pin with a short trace.
    - Two flex connectors must be arranged so most pass-through
      traces run straight across the top without crossings (otherwise
      you'll need to cut the GND pour to escape signals on bottom,
      which defeats the point).
    - Display bus speeds are modest (write strobes tens of MHz at
      most), so this is doable. USB D+/D− still want a continuous GND
      reference directly under them — route them with care.
- **Open:** mounting holes — does the board need to fit a specific
  spot inside the target enclosure, or is it free-floating with the
  flex cables and target 3-pin connector doing the mechanical work?
- **Open:** flex connector orientation (top vs bottom entry) — affects
  how the board sits relative to the display and main board, and
  whether the flex cables fold or run straight.

## References

- RP2350 datasheet §5.2 (boot sequence), §5.8 (UART boot), §5.9
  (IMAGE_DEF blocks)
- [display_notes.md](display_notes.md) — captured protocol details
- Pico 2 / Pico 2 W reference schematic for decoupling and USB
  termination values
