"""SKiDL netlist generator for the AQ LCD grab capture PCB.

Scope so far:
  - The two 39-pin flex connectors as a straight pass-through.
  - The RP2350A core, matching Raspberry Pi's official "RP2350 Minimal
    Board" reference design (reference/RP2350A_Minimal/) part-for-part:
    same symbol, same footprints, same cap/inductor values. This lets
    us paste in their proven layout decisions (placement, routing,
    decoupling distances) without value mismatches at import time.
  - 3-pin connector to the target main board (3V3, GND) for power.
  - USB recovery header + future-target-5V tap join (0Ω jumper).
  - Status LED on GPIO 25 + bring-up test points on GPIO 19-23.

Still to wire (later commits):
  - Flex bus tap from J2[2..24] to RP2350 GPIO0..18 (the capture
    fanout — kept out of this commit so the chip wiring can be
    reviewed in isolation).
  - Xiao ESP32-C6 castellated headers and ESP32-side signals
    (UART to RP2350, RP2350 RUN control, PIC32 reset).

Reference designators are intentionally LEFT for SKiDL to auto-assign
across runs (C1, C2, ..., R1, R2, ...). If you want to manually paste
schematic blocks from the RPi reference design into this project,
match by *value + topology*, not by reference designator.

Connectors:
  - J1: main-board side flex   (cable to the target PIC32 motherboard)
  - J2: display side flex      (cable to the LCD)
  - J3: 3-pin connector to target (3V3, GND, [PIC32 reset added later])
  - J4: 4-pin USB recovery header (VBUS, D+, D-, GND); VBUS doubles
        as the entry point for a future target-5V tap, joined to the
        ESP32 +5V net via a 0Ω jumper (SOD-123 footprint, swap for
        SS14 later if backfeed isolation becomes needed).

The flex connectors face opposite directions on the PCB, so J1[i]
lines up with J2[40 - i] as a straight trace across the board.
Each pin gets its own dedicated net — we do NOT merge nominally-
equivalent pins (multiple GNDs, multiple VCCs), because the target
flex pinout is partly guessed and merging could short two distinct
signals on the display.

Pin labels follow docs/display_notes.md (numbered against J2/display,
but they describe the signal on the net so they apply equally to J1's
mirrored numbering). Pins whose function is uncertain keep generic
`P<n>` names.
"""

import os

# SKiDL needs to know where KiCad's symbol libraries live so it can resolve
# stock symbols like Connector:Conn_01x39_Socket. Without this, SKiDL falls
# back to aq_lcd_grab_sklib.py (pin/footprint metadata only, no symbol) and
# KiCad warns "Footprint has no assigned symbol" on every netlist import.
os.environ.setdefault("KICAD9_SYMBOL_DIR", "/usr/share/kicad/symbols")

# Project-local sym-lib-table also maps:
#   MCU_RaspberryPi_RP2350 -> ./MCU_RaspberryPi_RP2350.kicad_sym
# and fp-lib-table maps:
#   RP2350_60QFN_minimal -> ./RP2350_60QFN_minimal.pretty/
# both copied from RPi's RP2350 minimal-board reference design.

from skidl import Part, Net, generate_netlist, lib_search_paths, KICAD9

# Tell SKiDL where to find our project-local RPi symbol file.
lib_search_paths[KICAD9].append(os.path.dirname(os.path.abspath(__file__)))


# =============================================================================
# Global power & signal nets
# =============================================================================
GND = Net("GND")
P3V3 = Net("+3V3")
P5V = Net("+5V")        # USB VBUS / future target 5V tap
USB_DP = Net("USB_DP")
USB_DM = Net("USB_DM")
P1V1 = Net("+1V1")      # RP2350 1.1V core, output of internal SMPS (RPi rail name)
VREG_LX = Net("VREG_LX")  # switching node, between RP2350 and inductor
XIN = Net("XIN")
XOUT = Net("XOUT")


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


def decouple(power_net, pin, ref, label, value="100n",
             footprint="Capacitor_SMD:C_0402_1005Metric"):
    """Drop a decoupling cap between `power_net` and GND, and tie `pin`
    to `power_net`. `ref` is the explicit refdes; `label` makes the
    SKiDL tag readable.
    """
    cap = C(value, ref, f"C_DECOUPLE_{label}", footprint=footprint)
    cap[1] += power_net
    cap[2] += GND
    pin += power_net


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

for i in range(1, 40):
    n = Net(FLEX_PIN_LABELS[i])
    n += J2[i], J1[40 - i]


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
# USB recovery header (4-pin 2.54 mm) + 5V join
# =============================================================================
J4 = Part("Connector", "Conn_01x04_Socket",
          footprint="Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical",
          ref="J4",
          tag="J4_USB_HEADER")
# Pin order on silk: VBUS, D+, D-, GND
J4[1] += P5V
J4[2] += USB_DP
J4[3] += USB_DM
J4[4] += GND

# 27Ω series resistors on D+/D− near the RP2350. Refdes R7/R8 match the
# corresponding parts in RPi's minimal-board reference schematic, so the
# component-level comparison stays 1:1.
USB_DP_CHIP = Net("USB_DP_CHIP")
USB_DM_CHIP = Net("USB_DM_CHIP")
R_DP = R("27", "R7", "R_USB_DP_SERIES")
R_DP[1] += USB_DP
R_DP[2] += USB_DP_CHIP
R_DM = R("27", "R8", "R_USB_DM_SERIES")
R_DM[1] += USB_DM
R_DM[2] += USB_DM_CHIP


# =============================================================================
# RP2350A
# =============================================================================
# Symbol and footprint are RPi's project-local versions (copied into pcb/
# from reference/RP2350A_Minimal/) so the part matches the official
# minimal-board reference design 1:1.
U1 = Part("MCU_RaspberryPi_RP2350", "RP2350_60QFN",
          footprint=("RP2350_60QFN_minimal:"
                     "RP2350-QFN-60-1EP_7x7_P0.4mm_EP3.4x3.4mm_ThermalVias"),
          ref="U1",
          tag="U1_RP2350")

# Power decoupling — refdes match RPi minimal-board reference 1:1 for
# the parts we share:
#   C1, C2     -> ADC_AVDD bulk + HF
#   C8, C11    -> extra +1V1 (DVDD) decoupling (RPi puts these on +1V1,
#                 not +3V3, despite the higher refdes — see reference)
#   C12-C17    -> 6× 100 nF for the 6× IOVDD pins (one each)
#   C18        -> VREG_AVDD decoupling
# QSPI_IOVDD (pin 54) and USB_OTP_VDD (pin 53) are not separately
# decoupled — we follow RPi's reference and rely on a nearby IOVDD cap
# (C12-C17) being placed close enough on the PCB to bypass them too.
# Just tie the pins to +3V3.
U1[54] += P3V3   # QSPI_IOVDD
U1[53] += P3V3   # USB_OTP_VDD

# 6× IOVDD on +3V3, one 100 nF per pin (RPi C12–C17).
IOVDD_PINS = [(1,  "C12", "IOVDD_1"),
              (11, "C13", "IOVDD_11"),
              (20, "C14", "IOVDD_20"),
              (30, "C15", "IOVDD_30"),
              (38, "C16", "IOVDD_38"),
              (45, "C17", "IOVDD_45")]
for pad_num, ref, label in IOVDD_PINS:
    decouple(P3V3, U1[pad_num], ref, label)

# VREG_AVDD (pin 46): analog supply for the internal regulator. RPi
# inserts a 33 Ω filter resistor (R5) between 3V3 and the VREG_AVDD pin,
# with a 100 nF cap (C18) to ground at the pin side.
VREG_AVDD = Net("VREG_AVDD")
R_VREG_AVDD = R("33", "R5", "R_VREG_AVDD_FILTER")
R_VREG_AVDD[1] += P3V3
R_VREG_AVDD[2] += VREG_AVDD
decouple(P3V3, U1[46], "C18", "VREG_AVDD")

# ADC_AVDD (pin 44): RPi reference uses 10 µF + 100 nF straight off 3V3
# (no ferrite bead). Matches their C1 (10 µF 0805) + C2 (100 nF).
C_ADC_BULK = C("10u", "C1", "C_ADC_AVDD_BULK",
               footprint="Capacitor_SMD:C_0805_2012Metric")
C_ADC_BULK[1] += P3V3
C_ADC_BULK[2] += GND
C_ADC_HF = C("100n", "C2", "C_ADC_AVDD_HF")
C_ADC_HF[1] += P3V3
C_ADC_HF[2] += GND
U1[44] += P3V3   # ADC_AVDD direct to 3V3 (no ferrite — matches reference)

# Internal SMPS for the 1.1V core (P1V1), exact-match to RPi reference:
#   VREG_VIN (49) <- 3V3, with 10 µF input cap (C5, 0805)
#   VREG_LX  (48) -> L1 (3.3 µH polarised AOTA-B201610S3R3-101-T) -> P1V1
#   VREG_PGND (47) -> GND
#   VREG_FB  (50) -> P1V1 (senses the filtered output)
#   P1V1 pins 23, 39 <- P1V1, with 2× 4.7 µF bulk caps near the inductor
#     plus 4.7 µF per P1V1 pin (C6, C7, C9, C10 in the reference).
C_VREG_VIN = C("10u", "C5", "C_VREG_VIN",
               footprint="Capacitor_SMD:C_0805_2012Metric")
C_VREG_VIN[1] += P3V3
C_VREG_VIN[2] += GND
U1[49] += P3V3

L1 = Part("Device", "L",
          value="3.3u",
          footprint="RP2350_60QFN_minimal:L_pol_2016",
          ref="L1",
          tag="L1_VREG")
L1[1] += VREG_LX
L1[2] += P1V1
U1[48] += VREG_LX
U1[47] += GND
U1[50] += P1V1    # VREG_FB senses P1V1 (the filtered output)

# RPi-style P1V1 bulk: 2× 4.7 µF near the chip on their custom "small_pads"
# 0402 footprint (C6, C7) plus one 4.7 µF per P1V1 pin (C9, C10).
SMALL_PADS_FP = "RP2350_60QFN_minimal:C_0402_1005Metric_small_pads"
C_P1V1_BULK_A = C("4.7u", "C6", "C_P1V1_BULK_A", footprint=SMALL_PADS_FP)
C_P1V1_BULK_A[1] += P1V1
C_P1V1_BULK_A[2] += GND
C_P1V1_BULK_B = C("4.7u", "C7", "C_P1V1_BULK_B", footprint=SMALL_PADS_FP)
C_P1V1_BULK_B[1] += P1V1
C_P1V1_BULK_B[2] += GND
decouple(VREG_AVDD, U1[23], "C9",  "P1V1_23", value="4.7u")
decouple(P1V1, U1[39], "C10", "P1V1_39", value="4.7u")
# Additional +1V1 HF decoupling caps to match RPi reference (C8, C11 — both
# 100 nF between +1V1 and GND, not tied to a specific U1 pin; they shunt
# the rail in general).
C8 = C("100n", "C8", "C_P1V1_HF_A")
C8[1] += P1V1
C8[2] += GND
C11 = C("100n", "C11", "C_P1V1_HF_B")
C11[1] += P1V1
C11[2] += GND

# GND on the exposed pad (symbol pin 61).
U1[61] += GND

# 12 MHz crystal: Abracon ABM8-272-T3, 3225 4-pin package. Load caps 15 pF
# each (RPi reference values, C3 + C4). RPi inserts a series damping
# resistor (R2, 1 kΩ) on the XOUT side — between U1's XOUT driver pin
# and the crystal/load-cap node. XOUT_DRIVE is the chip-side net, XOUT
# is post-resistor at the crystal terminal.
XOUT_DRIVE = Net("XOUT_DRIVE")
R_XOUT = R("1k", "R2", "R_XOUT_DAMP")
R_XOUT[1] += XOUT_DRIVE   # chip side (U1 pin 22)
R_XOUT[2] += XOUT         # crystal side
U1[22] += XOUT_DRIVE

Y1 = Part("Device", "Crystal_GND24",
          value="ABM8-272-T3",
          footprint="Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm",
          ref="Y1",
          tag="Y1_XTAL_12M")
Y1[1] += XIN
Y1[3] += XOUT
Y1[2] += GND
Y1[4] += GND
U1[21] += XIN

C_XIN = C("15p", "C3", "C_XIN_LOAD")
C_XIN[1] += XIN
C_XIN[2] += GND
C_XOUT = C("15p", "C4", "C_XOUT_LOAD")
C_XOUT[1] += XOUT
C_XOUT[2] += GND

# RUN (pin 26): 1 kΩ pull-up to 3V3 (matches RPi R4). The ESP32 will pull
# it low for reset in a later commit.
RUN = Net("RUN")
R_RUN = R("1k", "R4", "R_RUN_PULLUP")
R_RUN[1] += P3V3
R_RUN[2] += RUN
U1[26] += RUN

# UART boot strapping (no ESP32 GPIO needed — fully hard-wired):
#   QSPI_SS  (pin 60) -> GND   (selects BOOTSEL mode)
#   QSPI_SD1 (pin 59) -> 1 kΩ to 3V3   (selects UART within BOOTSEL)
# The 1 kΩ value matches the RUN pull-up choice (RPi uses 1 kΩ widely).
# QSPI_SD1 pull-up: we add this (RPi reference uses R6 for flash CS,
# which we don't have). Refdes R20 keeps us outside RPi's R1–R10 range
# so cross-comparison stays unambiguous.
U1[60] += GND
R_SD1 = R("1k", "R20", "R_QSPI_SD1_PULLUP")
R_SD1[1] += P3V3
R_SD1[2] += U1[59]

# QSPI_SD2 (pin 58) and QSPI_SD3 (pin 55) carry the 1 Mbaud UART boot
# protocol and later re-mux to hardware UART0 for runtime ESP32 comms.
UART_RP_TX = Net("UART_RP_TX")     # RP2350 TX (SD2) -> ESP32 RX
UART_RP_RX = Net("UART_RP_RX")     # RP2350 RX (SD3) <- ESP32 TX
U1[58] += UART_RP_TX
U1[55] += UART_RP_RX

# QSPI_SCLK (pin 56) and QSPI_SD0 (pin 57): unused (no flash); float at
# chip side. The bootrom doesn't drive SCLK without an SD command and
# SD0 floats in BOOTSEL mode.
NC_QSPI_SCLK = Net("NC_QSPI_SCLK")
NC_QSPI_SD0 = Net("NC_QSPI_SD0")
U1[56] += NC_QSPI_SCLK
U1[57] += NC_QSPI_SD0

# USB lines: chip-side pins, after the 27 Ω series resistors.
U1[51] += USB_DM_CHIP   # USB_DM
U1[52] += USB_DP_CHIP   # USB_DP

# SWD (pins 24/25) on a 3-pin 2.54mm header. Pinout follows the Raspberry
# Pi convention used on Pico-family boards (and the official Debug Probe):
#   Pin 1: SWCLK
#   Pin 2: GND
#   Pin 3: SWDIO
# 2.54mm pitch chosen over JST SH for easy hand-probing / pigtail use.
SWCLK = Net("SWCLK")
SWDIO = Net("SWDIO")
U1[24] += SWCLK
U1[25] += SWDIO

J5 = Part("Connector", "Conn_01x03_Pin",
          footprint="Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
          ref="J5",
          tag="J5_SWD_DEBUG")
J5[1] += SWCLK
J5[2] += GND
J5[3] += SWDIO


# =============================================================================
# Status LED on RP2350 GPIO 25 (pin 37) + current-limit resistor.
# Anode -> R (1 kΩ matches RPi) -> 3V3, cathode -> GPIO (active-low drive).
# GPIO sinks current, LED is off until firmware drives the pin low.
# =============================================================================
LED_STATUS = Net("LED_STATUS")
# Our addition (no LED in RPi reference). Refdes R21 keeps us outside
# RPi's R1–R10 range so cross-comparison stays unambiguous.
R_LED = R("1k", "R21", "R_LED_STATUS")
D_LED = Part("Device", "LED",
             value="GREEN",
             footprint="LED_SMD:LED_0603_1608Metric",
             ref="D1",
             tag="D1_LED_STATUS")
R_LED[1] += P3V3
R_LED[2] += D_LED[1]      # anode
D_LED[2] += LED_STATUS    # cathode -> GPIO 25
U1[37] += LED_STATUS


# =============================================================================
# Bring-up test points: GPIO 19–23 (RP2350 pins 31, 32, 33, 34, 35).
# 20/21 are the hardware UART1 TX/RX (F2 alt); 19/22/23 are general spare.
# =============================================================================
GPIO_TEST_PIN_MAP = [
    (19, 31, "TP1", "TP_GPIO19"),
    (20, 32, "TP2", "TP_GPIO20_UART1_TX"),
    (21, 33, "TP3", "TP_GPIO21_UART1_RX"),
    (22, 34, "TP4", "TP_GPIO22"),
    (23, 35, "TP5", "TP_GPIO23"),
]
for gpio_num, pad_num, ref, tag in GPIO_TEST_PIN_MAP:
    net = Net(f"GPIO{gpio_num}")
    U1[pad_num] += net
    tp = Part("Connector", "TestPoint",
              footprint="TestPoint:TestPoint_Pad_1.0x1.0mm",
              ref=ref,
              tag=tag)
    tp[1] += net

# Remaining unconnected RP2350 GPIOs (the capture bus pins 0–18 and ADC
# pins 26–29) are left dangling here; they'll get connected in the next
# commit (flex bus tap) and a possible ADC commit later.


# =============================================================================
# Xiao ESP32-C6 (DIP-mounted module)
# =============================================================================
# Footprint pads 1–14 wrap around the module in the silkscreen order. Pin
# functions (verified against the symbol shipped with the SnapEDA part):
#   1  GPIO0  (A0/D0)        -> RP2350 RUN drive (push-pull)
#   2  GPIO1  (A1/D1)        -> PIC32 reset (open-drain; never drive high)
#   3  GPIO2  (A2/D2)        free
#   4  GPIO21 (D3)           free
#   5  GPIO22 (D4/SDA)       free
#   6  GPIO23 (D5/SCL)       free
#   7  GPIO16 (D6/TX)        UART0 TX -> RP2350 UART RX (QSPI_SD3)
#   8  GPIO17 (D7/RX)        UART0 RX <- RP2350 UART TX (QSPI_SD2)
#   9  GPIO19 (D8/SCK)       free
#   10 GPIO20 (D9/MISO)      free
#   11 GPIO18 (D10/MOSI)     free
#   12 3V3                   power input (backfeed; bypasses Xiao LDO)
#   13 GND
#   14 5V                    VBUS / target 5V tap
#
# The module's onboard regulator is bypassed by backfeeding 3V3 on pad 12,
# so the USB-C connector on the Xiao itself is unusable for power while
# the capture PCB is connected to the target. Unplug the 3-pin target
# connector to power the Xiao from its own USB-C (see pcb_spec.md "Power").
U2 = Part("Connector_Generic", "Conn_01x14",
          footprint="esp32c6:XIAO-ESP32-C6-DIP",
          ref="U2",
          tag="U2_ESP32C6")

# Power & ground
U2[12] += P3V3
U2[13] += GND
U2[14] += P5V

# Reset / control outputs
U2[1] += RUN           # push-pull; RP2350 RUN has a 1 kΩ pull-up (R4)
U2[2] += PIC32_RESET   # open-drain; target board provides the pull-up

# UART0 to RP2350 bootrom (1 Mbaud) and runtime (any baud, F11 alt on RP2350)
U2[7] += UART_RP_RX    # ESP32 TX -> RP2350 RX (QSPI_SD3)
U2[8] += UART_RP_TX    # ESP32 RX <- RP2350 TX (QSPI_SD2)

# Free GPIOs (pads 3, 4, 5, 6, 9, 10, 11) — left dangling for now. Wire
# them in a later commit if we need extra ESP32 functions (e.g. status
# signal back to RP2350, sensor I2C, OTA-arm button).

# Bulk decoupling near pad 12 to absorb WiFi TX peaks locally so the
# target 3V3 rail doesn't see them as transients (pcb_spec.md Q6/Q7).
# Refdes C30/C31 sit outside the RPi C1–C18 range used for RP2350
# decoupling so the cross-reference with the reference schematic stays
# unambiguous.
C_ESP_BULK_A = C("22u", "C30", "C_ESP32_BULK_22U",
                 footprint="Capacitor_SMD:C_0805_2012Metric")
C_ESP_BULK_A[1] += P3V3
C_ESP_BULK_A[2] += GND
C_ESP_BULK_B = C("100u", "C31", "C_ESP32_BULK_100U",
                 footprint="Capacitor_SMD:C_1206_3216Metric")
C_ESP_BULK_B[1] += P3V3
C_ESP_BULK_B[2] += GND


generate_netlist(file_="aq_lcd_grab.net")
