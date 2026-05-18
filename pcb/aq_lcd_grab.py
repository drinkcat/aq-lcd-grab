"""SKiDL netlist generator for the AQ LCD grab capture PCB.

Current scope: just the two 39-pin flex connectors as a straight
pass-through.

  - J1: main-board side  (cable to the target PIC32 motherboard)
  - J2: display side     (cable to the LCD)

The two connectors face opposite directions on the PCB, so J1[i]
lines up with J2[40 - i] as a straight trace across the board —
that's the pass-through. Each pin gets its own dedicated net — we do
NOT merge nominally-equivalent pins (multiple GNDs, multiple VCCs),
because the target flex pinout is partly guessed and merging could
short two distinct signals on the display.

Pin labels follow the reverse-engineered table in docs/display_notes.md
(numbered against J2/display, but they describe the signal on the net
so they apply equally to J1's mirrored numbering). Pins whose function
is uncertain keep generic `P<n>` names.
"""

import os

# SKiDL needs to know where KiCad's symbol libraries live so it can resolve
# symbols like Connector_Generic:Conn_2Rows-39Pins to a real .kicad_sym file.
# Without this, SKiDL falls back to aq_lcd_grab_sklib.py (which only has
# pin/footprint metadata, no symbol) and KiCad warns "Footprint has no
# assigned symbol" on every netlist import.
os.environ.setdefault("KICAD9_SYMBOL_DIR", "/usr/share/kicad/symbols")

from skidl import Part, Net, generate_netlist

# -----------------------------------------------------------------------------
# Pin labels for the 39-pin flex, numbered against the display side (J2).
# Per docs/display_notes.md. Unknown/uncertain pins use a generic Pnn label.
# Where the same logical signal appears on multiple pins (e.g. several GND,
# several VCC), each pin gets a *distinct* net (GND_1, GND_18, ...) so we
# never accidentally short two pins on the display side.
# -----------------------------------------------------------------------------
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

# -----------------------------------------------------------------------------
# Connectors
# -----------------------------------------------------------------------------
# Explicit `tag` per part so SKiDL's identity is stable across runs.
# Without this, SKiDL assigns a fresh random tag each invocation and KiCad's
# netlist importer treats every run as "new parts," producing duplicate
# footprints on each re-import.
J1 = Part("Connector", "Conn_01x39_Socket",
          footprint="FH26W:FH26W39S03SHW60",
          tag="J1_FLEX_MAIN")
J2 = Part("Connector", "Conn_01x39_Socket",
          footprint="FH26W:FH26W39S03SHW60",
          tag="J2_FLEX_DISPLAY")

# -----------------------------------------------------------------------------
# Pass-through nets. The flex pinout in FLEX_PIN_LABELS is indexed against
# the display side (J2), so we use FLEX_PIN_LABELS[i] for J2[i] and the
# mirrored J1[40 - i] for the corresponding main-board pin.
# -----------------------------------------------------------------------------
for i in range(1, 40):
    n = Net(FLEX_PIN_LABELS[i])
    n += J2[i], J1[40 - i]

generate_netlist(file_="aq_lcd_grab.net")
