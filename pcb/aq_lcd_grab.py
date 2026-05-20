"""SKiDL netlist generator for the AQ LCD grab capture PCB.

Scope:
  - Two 39-pin flex connectors as a straight pass-through between the
    target main board and the LCD.
  - STM32F103C8T6 capture MCU (LQFP-48) tapping the display bus. See
    docs/pcb_spec.md for the rationale (JLCPCB basic-library part, no
    external SMPS, internal flash drops the UART-boot dance; chose
    F103 over the briefly-considered F030 for 20 kB SRAM + Cortex-M3).
  - Xiao ESP32-C6 (DIP-mount module) for WiFi + STM32 reflashing over
    USART1.
  - 3-pin connector to the target main board for 3V3 tap + PIC32 reset.
  - SWD header on the STM32 for bring-up / fallback flashing.
  - Status LED on STM32 PC13 (matches the Blue/Black Pill dev-board
    pinout so the same firmware blinks both; PB2 was the F030 draft
    choice but PB2 is the F103 BOOT1 latch and must stay pulled low)
    plus USART2 + spare-GPIO bring-up pads.

Connectors:
  - J1: main-board side flex   (cable to the target PIC32 motherboard)
  - J2: display side flex      (cable to the LCD)
  - J3: 3-pin connector to target (3V3, GND, PIC32 reset)
  - J5: 3-pin SWD header for STM32 (SWCLK / GND / SWDIO)

The flex connectors face opposite directions on the PCB, so J1[i] lines
up with J2[40 - i] as a straight trace across the board. Each pin gets
its own dedicated net — we do NOT merge nominally-equivalent pins
(multiple GNDs, multiple VCCs), because the target flex pinout is partly
guessed and merging could short two distinct signals on the display.

Pin labels follow docs/display_notes.md (numbered against J2/display,
but they describe the signal on the net so they apply equally to J1's
mirrored numbering). Pins whose function is uncertain keep generic
`P<n>` names.

Refdes scheme (renumbered for the STM32 design — no carryover from the
prior RP2350 draft):
  - C1–C4  : STM32 VDD decoupling (one 100 nF per VDD pin: pins 1/24/36/48)
  - C5     : STM32 VDDA HF (100 nF)
  - C6     : STM32 VDDA LF (1 µF)
  - C7     : STM32 3V3 bulk (4.7 µF)
  - C8     : STM32 NRST filter (100 nF, AN2586 §2.3.3 EMS)
  - C10–C11: ESP32 bulk caps (22 µF + 100 µF, WiFi TX transients)
  - R1     : BOOT0 pulldown (10 kΩ)
  - R2     : BOOT1/PB2 pulldown (10 kΩ — must be 0 at reset, AN2586 §4.1)
  - R3     : STM32 status LED series resistor (1 kΩ)
  - D1     : STM32 status LED
"""

import os

# SKiDL needs to know where KiCad's symbol libraries live so it can resolve
# stock symbols like Connector:Conn_01x39_Socket and MCU_ST_STM32F1:STM32F103C8Tx.
os.environ.setdefault("KICAD9_SYMBOL_DIR", "/usr/share/kicad/symbols")

from skidl import Part, Net, generate_netlist, lib_search_paths, KICAD9

# Project-local libraries (esp32c6 footprint, test_points). The RP2350-era
# MCU_RaspberryPi_RP2350 / RP2350_60QFN_minimal libraries are no longer
# used by this design but remain in sym-lib-table / fp-lib-table for now;
# they can be cleaned up separately.
lib_search_paths[KICAD9].append(os.path.dirname(os.path.abspath(__file__)))


# =============================================================================
# Global power & signal nets
# =============================================================================
GND = Net("GND")
P3V3 = Net("+3V3")
P5V = Net("+5V")        # Xiao USB-C VBUS *and* the optional target 5V
                        # tap header (J4). Don't have both live at the
                        # same time without an OR'ing scheme.


# =============================================================================
# Helpers
# =============================================================================
def C(value, ref, tag, footprint="Capacitor_SMD:C_0402_1005Metric"):
    """0402 ceramic cap by default. `ref` is the explicit refdes (e.g.
    "C1"); `tag` is the SKiDL identity for stable cross-run matching.
    """
    return Part("Device", "C", value=value, footprint=footprint,
                ref=ref, tag=tag)


def R(value, ref, tag, footprint="Resistor_SMD:R_0402_1005Metric"):
    return Part("Device", "R", value=value, footprint=footprint,
                ref=ref, tag=tag)


# =============================================================================
# 39-pin flex pin labels (display side, J2 numbering)
# =============================================================================
FLEX_PIN_LABELS = {
    1:  "GND_1",
    2:  "DB0",
    3:  "DB1",
    4:  "DB2",
    5:  "DB3",
    6:  "DB4",
    7:  "DB5",
    8:  "DB6",
    9:  "DB7",
    10: "DB8",
    11: "DB9",
    12: "DB10",
    13: "DB11",
    14: "DB12",
    15: "DB13",
    16: "DB14",
    17: "DB15",
    18: "GND_18",
    19: "GND_19",
    20: "P20_RD",      # held high, likely RD (tied) or RST (released)
    21: "P21",         # unknown — TE / RST / NC
    22: "WR",
    23: "DC",
    24: "CS",
    25: "P25",
    26: "P26",
    27: "P27",
    28: "P28",
    29: "P29",
    30: "P30",
    31: "P31",
    32: "P32",
    33: "VCC_33",
    34: "P34",
    35: "VCC_35",
    36: "VCC_36",
    37: "VCC_37",
    38: "P38",
    39: "P39",
}


# =============================================================================
# Flex connectors and pass-through (J1 ↔ J2)
# =============================================================================
J1 = Part("Connector", "Conn_01x39_Socket",
          footprint="FH26W:FH26W39S03SHW60",
          ref="J1",
          tag="J1_FLEX_MAIN")
J2 = Part("Connector", "Conn_01x39_Socket",
          footprint="FH26W:FH26W39S03SHW60",
          ref="J2",
          tag="J2_FLEX_DISPLAY")

flex_nets = {}   # label -> Net, so the bus tap below can look signals up
for i in range(1, 40):
    label = FLEX_PIN_LABELS[i]
    n = Net(label)
    n += J2[i], J1[40 - i]
    flex_nets[label] = n

# Tie each suspected-GND flex pin to board GND through a dedicated
# 0402 0 Ω jumper. One jumper per logical pin grounds both flex
# terminals because they share the pass-through net.
# Populated by default; depopulate with hot air if scope-check shows
# the pin isn't actually GND on the target side. See pcb_spec.md
# "Flex GND tie".
GND_TIE_REFS = [
    ("GND_1",  "R10"),
    ("GND_18", "R11"),
    ("GND_19", "R12"),
]
for net_label, ref in GND_TIE_REFS:
    r = R("0", ref, f"R_FLEX_{net_label}_TIE")
    r[1] += flex_nets[net_label]
    r[2] += GND


# =============================================================================
# 3-pin power connector to target main board
# (3V3 tap, GND, PIC32 reset)
# =============================================================================
J3 = Part("Connector", "Conn_01x03_Socket",
          footprint="Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
          ref="J3",
          tag="J3_AIRRUN_POWER")
PIC32_RESET = Net("PIC32_RESET")   # open-drain from ESP32 (see ESP32 section)
J3[1] += P3V3
J3[2] += GND
J3[3] += PIC32_RESET


# =============================================================================
# Optional target 5V tap header (J4)
# =============================================================================
# 2-pin 2.54 mm header wired to GND + the +5V net (Xiao USB-C VBUS).
# Unpopulated by default; only soldered when the target 3V3 tap
# (Q6) turns out not to have enough headroom for Xiao WiFi peaks
# and we need to fall back to powering the Xiao through its onboard
# LDO from the target's 5V rail instead. See pcb_spec.md "Power".
J4 = Part("Connector", "Conn_01x02_Socket",
          footprint="Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
          ref="J4",
          tag="J4_AIRRUN_5V_TAP")
J4[1] += P5V
J4[2] += GND


# =============================================================================
# STM32F103C8T6 (LQFP-48) — capture MCU
# =============================================================================
# Pin map verified against DS5319 Table 5 (see docs/pcb_spec.md "Pin
# map (proposal)"). LQFP-48 footprint from KiCad stock library; no
# exposed pad. The F103 LQFP-48 pinout matches F030 LQFP-48 for every
# pin we use, except: pin 1 is VBAT (not VDD), pin 35/36 is an extra
# VSS/VDD pair (replaces F030's PF6/PF7), and PB2 is the BOOT1 latch
# (must be held low at reset; cannot drive the LED).
U1 = Part("MCU_ST_STM32F1", "STM32F103C8Tx",
          footprint="Package_QFP:LQFP-48_7x7mm_P0.5mm",
          ref="U1",
          tag="U1_STM32")

# --- Power pins ---------------------------------------------------------
# VDD: pins 24, 36, 48. VBAT: pin 1 (tied to VDD — no battery).
# VDDA: pin 9. VSS: pins 23, 35, 47. VSSA: pin 8.
for pad_num in (1, 24, 36, 48):
    U1[pad_num] += P3V3
U1[9] += P3V3       # VDDA tied to VDD (no separate analog rail)
U1[8] += GND        # VSSA
for pad_num in (23, 35, 47):
    U1[pad_num] += GND

# Decoupling per AN2586 §2.2:
#   - 100 nF per VDD pin (×3: pins 24, 36, 48) + 1× 100 nF on VBAT
#     (pin 1, tied to VDD as we have no battery).
#   - 100 nF + 1 µF on VDDA (close to pin 9).
#   - 4.7 µF bulk on the 3V3 rail near the chip.
C1 = C("100n", "C1", "C_STM32_VBAT_PIN1")
C1[1] += P3V3; C1[2] += GND
C2 = C("100n", "C2", "C_STM32_VDD_PIN24")
C2[1] += P3V3; C2[2] += GND
C3 = C("100n", "C3", "C_STM32_VDD_PIN36")
C3[1] += P3V3; C3[2] += GND
C4 = C("100n", "C4", "C_STM32_VDD_PIN48")
C4[1] += P3V3; C4[2] += GND
C5 = C("100n", "C5", "C_STM32_VDDA_HF")
C5[1] += P3V3; C5[2] += GND
C6 = C("1u", "C6", "C_STM32_VDDA_LF")
C6[1] += P3V3; C6[2] += GND
C7 = C("4.7u", "C7", "C_STM32_3V3_BULK")
C7[1] += P3V3; C7[2] += GND

# --- Reset / boot ------------------------------------------------------
# NRST (pin 7): internal pull-up; AN2586 §2.3.3 recommends 100 nF to
# GND for EMS. ESP32 drives this open-drain (see ESP32 section).
NRST = Net("NRST")
U1[7] += NRST
C_NRST = C("100n", "C8", "C_STM32_NRST_FILTER")
C_NRST[1] += NRST; C_NRST[2] += GND

# BOOT0 (pin 44): 10 kΩ pull-down so the chip boots from user flash by
# default. ESP32 drives push-pull HIGH to enter the ROM bootloader.
BOOT0 = Net("BOOT0")
U1[44] += BOOT0
R_BOOT0 = R("10k", "R1", "R_BOOT0_PULLDOWN")
R_BOOT0[1] += BOOT0; R_BOOT0[2] += GND

# BOOT1 / PB2 (pin 20): on F103 the BOOT1 latch is muxed with PB2, and
# both boot modes we use require BOOT1=0 (AN2586 §4.1). A 10 kΩ
# pull-down to GND on PB2 holds the latch low before firmware
# configures the pin. PB2 is **not** wired as the status LED here
# (the F030 draft did; that would have forced BOOT1=1 at reset).
BOOT1 = Net("BOOT1")
U1[20] += BOOT1
R_BOOT1 = R("10k", "R2", "R_BOOT1_PULLDOWN")
R_BOOT1[1] += BOOT1; R_BOOT1[2] += GND

# --- SWD header (3-pin) ------------------------------------------------
# Pinout mirrors the prior board's convention: pin 1 SWCLK, pin 2 GND,
# pin 3 SWDIO. 2.54 mm pitch for hand-probing.
SWDIO = Net("SWDIO")
SWCLK = Net("SWCLK")
U1[34] += SWDIO     # PA13
U1[37] += SWCLK     # PA14

J5 = Part("Connector", "Conn_01x03_Pin",
          footprint="Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
          ref="J5",
          tag="J5_SWD_DEBUG")
J5[1] += SWCLK
J5[2] += GND
J5[3] += SWDIO

# --- USART1 (PA9 TX / PA10 RX): ESP32 ↔ STM32 link --------------------
# Same pins for the ROM bootloader (AN2606 — no PB6/PB7 remap scan) and
# for the runtime data link. Never multiplexed with data-bus capture.
#
# Each line is split into STM32-side and ESP32-side halves by a series
# 0 Ω jumper, so the link can be broken during bring-up to drive either
# side from an external USB-UART probe without the other side pulling
# on the line. Place the jumpers near the STM32 (short stubs on the
# probe-friendly ESP32 side).
UART_STM_TX = Net("UART_STM_TX")        # STM32 TX (PA9) -> jumper -> ESP32 RX
UART_STM_RX = Net("UART_STM_RX")        # STM32 RX (PA10) <- jumper <- ESP32 TX
UART_ESP_RX = Net("UART_ESP_RX")        # ESP32-side of TX line
UART_ESP_TX = Net("UART_ESP_TX")        # ESP32-side of RX line
U1[30] += UART_STM_TX                   # PA9
U1[31] += UART_STM_RX                   # PA10

R_UART_TX = R("0", "R13", "R_UART_TX_BREAK")
R_UART_TX[1] += UART_STM_TX; R_UART_TX[2] += UART_ESP_RX
R_UART_RX = R("0", "R14", "R_UART_RX_BREAK")
R_UART_RX[1] += UART_STM_RX; R_UART_RX[2] += UART_ESP_TX


# =============================================================================
# Display capture tap (flex bus -> STM32)
# =============================================================================
# Capture path reads `GPIOA->IDR` and `GPIOB->IDR` as whole 16-bit
# ports via two DMA channels (see docs/pcb_spec.md Q12). The *logical*
# DBn identity of any given STM32 pad is therefore not load-bearing —
# the host decoder applies a permutation, derived from this very table,
# that undoes whatever order we pick here. Routing is free to swap DBn
# labels between entries within the same port (PA0..PA7 stay PA pins,
# PB-pins stay PB pins); changing the table here is the only thing
# needed to retarget the host decoder. See docs/pcb_spec.md
# "Pin allocation (rules, not a fixed map)" and Q17.
#
# What is *forced* by silicon:
#   - WR -> PA12 (TIM1_ETR; no AFIO remap on F103).
#   - DC and CS must be on PB pins so they ride along in the same
#     `GPIOB->IDR` DMA read as the PB-half of the data bus.
#   - 8 data bits must land on PA pins, 8 on PB pins; mixing across
#     ports would require an extra cross-port merge on the host.
#
# Everything else (which specific PA pin gets DB0, which PB pin gets
# DC, etc.) is router's choice.
CAPTURE_TAP = [
    # (STM32 pad #, flex net label)   STM32 pin   notes
    # --- PA port (data-bus PA-half) -------------------------------------
    (10, "DB12"),    # PA0
    (11, "DB10"),    # PA1
    (12, "DB8"),     # PA2
    (13, "DB6"),     # PA3
    (14, "DB4"),     # PA4
    (15, "DB2"),     # PA5
    (16, "DB0"),     # PA6
    (17, "DB1"),     # PA7
    # --- PA port (capture trigger, forced) ------------------------------
    (33, "WR"),      # PA12       TIM1_ETR
    # --- PB port (data-bus PB-half + control) ---------------------------
    (18, "DB5"),     # PB0
    (19, "DB9"),     # PB1
    (21, "DB13"),     # PB10
    (22, "DB3"),     # PB11
    (25, "DB7"),    # PB12
    (26, "DB11"),    # PB13
    (27, "DB15"),    # PB14
    (28, "DC"),      # PB15       command/data framing line
    (45, "DB14"),    # PB8
    (46, "CS"),      # PB9        chip select
]
for pad, label in CAPTURE_TAP:
    U1[pad] += flex_nets[label]


# =============================================================================
# Status LED on STM32 PC13 (pin 2)
# =============================================================================
# Matches the Blue/Black Pill dev-board pinout: same pin, same anode→
# 1 kΩ→3V3 / cathode→GPIO active-low topology. So a firmware blink on
# PC13 lights both the dev-board LED and ours, no per-target #ifdef.
#
# PC13 is in the F103 backup domain (low-drive, 3 mA max sink/source,
# 2 MHz toggle, not 5V tolerant per DS5319). All within spec for a
# <2 mA LED at sub-Hz rates.
LED_STATUS = Net("LED_STATUS")
R_LED = R("1k", "R3", "R_LED_STATUS")
D_LED = Part("Device", "LED",
             value="GREEN",
             footprint="LED_SMD:LED_0603_1608Metric",
             ref="D1",
             tag="D1_LED_STATUS")
R_LED[1] += P3V3
R_LED[2] += D_LED[1]      # anode
D_LED[2] += LED_STATUS    # cathode -> PC13
U1[2] += LED_STATUS


# =============================================================================
# Bring-up test points on STM32 free pins
# PB3, PB4, PB5 are free GPIOs intended as scope-probe points during
# bring-up. Note: on F103 PB3/PB4 default to JTAG (PB3=JTDO,
# PB4=NJTRST); firmware must write `AFIO_MAPR.SWJ_CFG=010` early to
# disable JTAG-DP and free them as plain GPIO (we use SWD only, not
# JTAG). PB5 has no AF default to worry about.
# Bring-up serial console uses SWO via the SWD probe (Q14), not a
# dedicated UART pad cluster.
TEST_POINTS = [
    (39, "TP1", "TP_PB3"),   # JTDO — needs JTAG disabled
    (40, "TP2", "TP_PB4"),   # NJTRST — needs JTAG disabled
    (41, "TP3", "TP_PB5"),
]
for pad_num, ref, tag in TEST_POINTS:
    tp = Part("Connector", "TestPoint",
              footprint="test_points:TestPoint_Pad_0.5x0.5mm",
              ref=ref,
              tag=tag)
    net = Net(tag.replace("TP_", ""))
    U1[pad_num] += net
    tp[1] += net


# =============================================================================
# Xiao ESP32-C6 (DIP-mounted module)
# =============================================================================
# Footprint pads 1–14 wrap around the module in the silkscreen order.
#   1  GPIO0  (A0/D0)        unused
#   2  GPIO1  (A1/D1)        unused
#   3  GPIO2  (A2/D2)        unused
#   4  GPIO21 (D3)           unused
#   5  GPIO22 (D4/SDA)       unused
#   6  GPIO23 (D5/SCL)       unused
#   7  GPIO16 (D6/TX)        UART0 TX -> STM32 USART1 RX (PA10)
#   8  GPIO17 (D7/RX)        UART0 RX <- STM32 USART1 TX (PA9)
#   9  GPIO19 (D8/SCK)       STM32 NRST (open-drain; NRST has internal pull-up)
#   10 GPIO20 (D9/MISO)      PIC32 reset (open-drain; never drive high)
#   11 GPIO18 (D10/MOSI)     STM32 BOOT0 (push-pull, drive high to enter bootloader)
#   12 3V3                   power input (backfeed; bypasses Xiao LDO)
#   13 GND
#   14 5V                    Xiao USB-C VBUS (no target 5V tap)
#
# Single-edge routing: connected pins 7–14 land on the back edge as
# drawn. Pads 1–6 are intentionally left unconnected. To reflash the
# ESP32 standalone, unplug the 3-pin target connector so the Xiao can
# run from its own USB-C without back-driving the target 3V3 rail.
U2 = Part("Connector_Generic", "Conn_01x14",
          footprint="esp32c6:XIAO-ESP32-C6-DIP",
          ref="U2",
          tag="U2_ESP32C6")

# Power & ground
U2[12] += P3V3
U2[13] += GND
U2[14] += P5V

# USART1 to STM32 (ROM bootloader + runtime; up to 115200+)
U2[7] += UART_ESP_TX    # ESP32 TX -> 0 Ω -> STM32 RX (PA10)
U2[8] += UART_ESP_RX    # ESP32 RX <- 0 Ω <- STM32 TX (PA9)

# Reset / boot control
U2[9]  += NRST          # GPIO19 — open-drain (NRST has internal pull-up)
U2[10] += PIC32_RESET   # GPIO20 — open-drain; target provides the pull-up
U2[11] += BOOT0         # GPIO18 — push-pull, drive HIGH to enter bootloader

# Bulk decoupling near Xiao 3V3 pad to absorb WiFi TX peaks locally so
# the target 3V3 rail doesn't see them as transients (pcb_spec.md Q6/Q7).
C_ESP_BULK_A = C("22u", "C10", "C_ESP32_BULK_22U",
                 footprint="Capacitor_SMD:C_0805_2012Metric")
C_ESP_BULK_A[1] += P3V3
C_ESP_BULK_A[2] += GND
C_ESP_BULK_B = C("100u", "C11", "C_ESP32_BULK_100U",
                 footprint="Capacitor_SMD:C_1206_3216Metric")
C_ESP_BULK_B[1] += P3V3
C_ESP_BULK_B[2] += GND


generate_netlist(file_="aq_lcd_grab.net")
