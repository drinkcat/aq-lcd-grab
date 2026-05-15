# target device Display Reverse Engineering

## Goal

Capture the MCU → display protocol on the target device air quality monitor and
decode the partial-update rectangle commands so we can mirror / log the display
contents.

## Device

- **target device** — Taiwanese air quality monitor (PM2.5, TVOC, CO2, temp, humidity)
- **MCU**: PIC32MM (low-power MIPS, typically 25 MHz max, 16–32 KB RAM)
- **Display**: 3.5" color TFT, no touch, simple UI with colored polygons + text

## Display Cable (FPC)

- Type: **FFC, Type A** (contacts on one side, same side both ends)
- **39 pins**
- **0.3mm pitch**
- **~12mm wide**

Adapter for breadboard probing: a generic "FPC Adapter Board 39Pin: DuPont
2.0/2.54mm to FPC 39P, Pitch 0.3mm/DIP LVDS MIPI Board".

Compatible PCB connector: Crystalfontz **CS030Z39G-A0** (or LCSC equivalent —
search `FPC 0.3mm 39P ZIF top contact`).

## Suspected Display Controller

Unidentified, but based on behavior almost certainly an **ILI9488** or
**ST7796S** family chip (or compatible clone):

- Built-in GRAM so partial updates work despite tiny MCU RAM
- 16-bit 8080 parallel MCU interface
- 320×480 resolution (assumed — to be confirmed from capture)

A 39-pin 3.5" Tianma panel exists (TM035KDH03-39) but uses RGB interface, not
8080 — probably **not** the same panel. The target's panel is likely a generic OEM
unit; no public datasheet found.

## Estimated Pinout

Probed signals (note: pin 1/39 orientation may be flipped — adjust if so).

| Pin | Signal (observed) | Likely function |
|-----|-------------------|-----------------|
| 1 | GND (continuity) | GND |
| 2–6 | high-freq pulses | DB0–DB4 (data low bits) |
| 7–12 | pulses, sometimes lingering 3.3V | DB5–DB10 |
| 13–17 | slower pulses | DB11–DB15 (data high bits) |
| 18 | GND (continuity) | GND |
| 19 | GND (continuity) | GND |
| 20 | held high | RD (tied high, not used) or RST (released) |
| 21 | unknown | TE / RST / NC |
| 22 | 500ns low pulses every ~1.5µs | **WR** (write strobe, ~667 kHz word rate) |
| 23 | ~500ns low pulse every ~10µs | **D/C (RS)** (command vs data) |
| 24 | ~200ns low pulses every ~1.5µs | **CS** per-word, or strobe |
| 25–32 | unknown | LED+/LED−, extra GND, RST, TE, NC |
| 33 | 3.3V (continuity) | VCC/VDDI |
| 34 | unknown | likely VCC/GND |
| 35 | 3.3V (continuity) | VCC/IOVCC |
| 36 | 3.3V (continuity) | VCC |
| 37 | 3.3V (continuity) | VCC |
| 38 | unknown | likely VCC/GND |
| 39 | unknown | likely GND |

### Sanity-check math

- WR at 667 kHz × 5 ms burst ≈ **3,300 words/burst** ≈ 6.6 KB per burst
- 320×480×2 = 300 KB full frame → each burst paints ~2% of the screen
- Plausible for redrawing a few digits or polygon backgrounds per update
- Update cadence: **5 ms bursts every ~1 s**

### To confirm with capture

- [ ] Bus width (16 vs 8 bit — bet is 16)
- [ ] WR vs CS phase relationship (which pulse contains which)
- [ ] DC behavior at burst boundaries → identify command bytes
- [ ] Identify Set Column Address (`0x2A`), Set Row Address (`0x2B`), Memory
      Write (`0x2C`) commands and their arguments
- [ ] Read Display ID (`0x04`) if the PIC32MM ever issues it on startup

## Capture Hardware Plan

### Target: Raspberry Pi Pico 2 W (RP2350)

Chosen because:
- PIO is ideal for parallel bus capture (independent of CPU)
- 520 KB RAM (vs 264 on Pico W) — plenty of headroom
- Identical pinout to Pico W, same wireless chip (CYW43439)
- 12 PIO state machines across 3 blocks
- WiFi for streaming captures to laptop (or USB CDC also works)

### Pin assignment

The data bus needs **consecutive GPIOs** for `in pins, 16` in PIO.
GPIO 23, 24, 25, 29 are reserved for WiFi chip on Pico W/2W.

| Pico GPIO | Function |
|-----------|----------|
| GPIO 0–15 | DB0–DB15 (16-bit data bus) |
| GPIO 16 | WR (capture trigger) |
| GPIO 17 | D/C (RS) |
| GPIO 18 | CS |
| GPIO 19 | RD or RST or TE (spare) |
| GPIO 20 | UART TX (debug) |
| GPIO 21 | UART RX (debug) |
| GPIO 22, 26–28 | spare |

### Capture strategy

Data rate is tiny — ~6.6 KB/burst, ~7 KB/s average. No DMA needed strictly,
but PIO + DMA is the clean design:

1. **PIO state machine 1**: waits on WR falling edge, samples
   `{DC, CS, DB15..DB0}` (18 bits = pad to 32 bit word in FIFO), pushes to FIFO
2. **DMA**: drains PIO FIFO to a ring buffer in RAM
3. **Main loop**: detects idle period between bursts, packages the burst, sends
   over WiFi (UDP) or USB CDC to host
4. **Host (Python)**: maintains framebuffer model, applies window-set + GRAM
   write commands, renders to a window for live mirror

### Sampling considerations

- WR low pulse is ~500 ns wide → sample on falling edge, data should be stable
- PIO at 150 MHz = 6.67 ns per cycle, plenty of headroom
- Worth capturing on **WR rising edge** instead (data definitely valid at end
  of strobe, matches 8080 spec) — TBD based on traces

## Firmware Plan

### Language: Rust + Embassy (`embassy-rp`)

Rationale: prior experience with Embassy on ESP32-C6, good PIO support via
`pio` and `pio-proc` crates, async makes WiFi + USB serial concurrent code
clean.

### Crates

- `embassy-executor`, `embassy-rp`, `embassy-time`
- `pio`, `pio-proc` — PIO assembly
- `cyw43`, `cyw43-pio` — WiFi
- `embassy-net` — TCP/UDP
- `defmt` + `defmt-rtt` — logging over RTT
- `panic-probe` — panic handler

### Modules to write

- [ ] `pio_capture.rs` — PIO state machine assembly, FIFO setup
- [ ] `dma_buffer.rs` — ring buffer, burst boundary detection
- [ ] `protocol.rs` — decode framing (command vs data based on DC)
- [ ] `wifi_stream.rs` — UDP sender to host
- [ ] `main.rs` — task spawning

### Open questions for firmware

- Should we decode the protocol on the Pico, or just stream raw
  (cmd byte, data...) tuples and decode on host? **→ Decode on host** for
  flexibility, Pico just packages and forwards.
- Buffer size for one burst? 6.6 KB nominal, allocate 32 KB to be safe.
- How to mark burst boundaries? Detect idle period (no WR for > 1 ms), then
  flush.

## Host-side Plan (out of scope for first firmware pass)

- Python script receives UDP packets containing raw (DC, data) tuples
- Decode ILI9488/ST7796S command set
- Maintain a NumPy framebuffer, apply window-set + GRAM writes
- Render with matplotlib or pygame as live mirror
- Log captures to file for offline analysis

## References

- ILI9488 datasheet (for command set reference, `0x2A`/`0x2B`/`0x2C` etc.)
- ST7796S datasheet (for command set comparison)
- Embassy RP examples: https://github.com/embassy-rs/embassy/tree/main/examples/rp235x
- PIO programming: RP2350 datasheet chapter on PIO
