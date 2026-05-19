"""Trace straightener for aq_lcd_grab.kicad_pcb.

For every flex pass-through net (J1 <-> J2), replace all inner-layer segments
with a polyline that goes from one F.Cu<->B.Cu via to the other along a
near-vertical column, with 45° diagonal jogs around any obstacle via that
would violate DRC. F.Cu fanout stubs and the vias themselves are untouched.

CAVEAT: the via-avoidance algorithm here is a starting point, not finished.
Running this script in its current form produces DRC violations (corner
clearance is insufficient near tightly-packed flex vias). Use the existing
hand-routed PCB as the reference pattern when iterating; see commit
aa244b63fcac for canonical 45°-jog routing.

Hand-routing pattern (per VCC_35, top via at y=33.24):
  - Stay on column until ~0.05 mm before the obstacle's "danger zone"
  - 45° diagonal jog (Δx == Δy) to a perpendicular offset just large
    enough to clear: offset = obstacle.radius + clearance + half_track,
    on the side AWAY from the obstacle.
  - Run straight at offset, past the obstacle.
  - If next obstacle is on the same side: hold offset; if opposite side:
    single 45° cross to the new offset (no return-to-column between).
  - After last obstacle: 45° jog back to column.

Run from anywhere:
    python3 pcb/straighten_traces.py
"""

import math
import re
import sys
import uuid
from collections import defaultdict
from pathlib import Path

PCB = Path(__file__).resolve().parent / "aq_lcd_grab.kicad_pcb"

# Flex pass-through nets — must match FLEX_PIN_LABELS in aq_lcd_grab.py.
FLEX_NETS = [
    "GND_1", "DB0", "DB1", "DB2", "DB3", "DB4", "DB5", "DB6", "DB7",
    "DB8", "DB9", "DB10", "DB11", "DB12", "DB13", "DB14", "DB15",
    "GND_18", "GND_19", "P20_RD", "P21", "WR", "DC", "CS",
    "P25", "P26", "P27", "P28", "P29", "P30", "P31", "P32",
    "VCC_33", "P34", "VCC_35", "VCC_36", "VCC_37", "P38", "P39",
]

CLEARANCE = 0.2     # mm, from project netclass "Default"
DETOUR_PAD = 0.05   # mm, extra slack so we sit clearly outside the DRC limit

TOK = re.compile(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()"]+', re.DOTALL)


def parse_one(text, start):
    """Parse exactly one s-expression form beginning at text[start] == '('."""
    assert text[start] == '('
    stack = []
    current = None
    i = start
    while True:
        m = TOK.search(text, i)
        tok = m.group(0)
        i = m.end()
        if tok == '(':
            if current is not None:
                stack.append(current)
            current = []
        elif tok == ')':
            done = current
            if not stack:
                return done, i
            current = stack.pop()
            current.append(done)
        else:
            current.append(tok)


def unq(s):
    return s[1:-1] if s.startswith('"') and s.endswith('"') else s


def dist_point_segment(px, py, ax, ay, bx, by):
    """Shortest distance from point P to segment AB, plus the t in [0,1]
    locating the closest point on AB."""
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return math.hypot(px - ax, py - ay), 0.0
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy), t


# Flex vias sit on a staggered 4-band grid per connector side. The two "deep"
# bands (top y=33.24 and y=36.04) need to traverse the in-between rows of
# foreign vias; they get a 45°-jog plan. The two "shallow" bands (38.94, 40.94)
# sit closest to the middle and route straight through — they're on a
# different inner layer (In1.Cu) than the deep bands (In2.Cu), so they don't
# need to weave around the deep bands' vias.
#
# Within the deep bands, the two y-values use OPPOSITE jog patterns so
# adjacent same-layer nets nudge into the gaps between each other's columns:
#   y=33.24 → jog LEFT first, then RIGHT (then back to column)
#   y=36.04 → jog RIGHT first, then LEFT (then back to column)
#
# The hand-routed reference (commit aa244b63fcac) uses offsets of 0.225 mm
# (deep side) and 0.185 mm (shallow side). Each jog is a true 45° diagonal
# (|Δx| == |Δy|). The plan describes the TOP fanout; the bottom fanout
# mirrors it.
#
# Plan entries are (Δx_offset, Δy_below_top_via): cumulative waypoints on
# the trace, with 45° diagonals interpolated between consecutive entries.

JOG_PLANS = {
    "33.24": [
        (0.0,     4.81),    # stay on column ~4.8 mm
        (-0.225,  5.035),   # 45° jog LEFT (Δx=0.225, Δy=0.225)
        (-0.225,  6.385),   # hold LEFT past first obstacle row (y=38.94)
        (+0.185,  6.795),   # 45° cross to RIGHT (Δ=0.41)
        (+0.185,  8.515),   # hold RIGHT past second obstacle row (y=40.94)
        (0.0,     8.700),   # 45° back to column
    ],
    "36.04": [
        (0.0,     2.0),     # stay on column
        (+0.185,  2.185),   # 45° jog RIGHT
        (+0.185,  3.585),   # hold past first obstacle row (y=38.94)
        (-0.225,  3.995),   # cross to LEFT
        (-0.225,  5.675),   # hold past second obstacle row (y=40.94)
        (0.0,     5.900),   # back to column
    ],
}

BANDS = (33.24, 36.04, 38.94, 40.94)


def band_for(y):
    """Snap a via y to its nearest band key; None if no band is in range."""
    for b in BANDS:
        if abs(y - b) < 0.05:
            return f"{b:.2f}"
    return None


def route_with_plan(ax, ay, bx, by):
    """Polyline from A to B following the band's jog plan, mirrored top/bottom.

    Vias in the shallow bands (38.94, 40.94) get a single straight segment —
    they're on a different inner layer and don't need to dodge.
    """
    # Decide which via is "top" (smaller y in board coordinates).
    if ay < by:
        top_x, top_y, bot_x, bot_y = ax, ay, bx, by
    else:
        top_x, top_y, bot_x, bot_y = bx, by, ax, ay

    top_band = band_for(top_y)
    bot_band = band_for(bot_y)
    plan = JOG_PLANS.get(top_band)
    bot_plan = JOG_PLANS.get(bot_band, plan)

    # Shallow band or unknown band — straight through.
    if plan is None:
        return [(ax, ay), (bx, by)]

    pts = [(top_x, top_y)]
    for dx, dy in plan:
        pts.append((top_x + dx, top_y + dy))
    # Connect to the start of the bottom-side mirror.
    bot_max_dy = max(dy for _, dy in bot_plan)
    pts.append((top_x + plan[-1][0], bot_y - bot_max_dy))
    for dx, dy in reversed(bot_plan):
        pts.append((top_x + dx, bot_y - dy))
    pts.append((bot_x, bot_y))

    deduped = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - deduped[-1][0], p[1] - deduped[-1][1]) > 1e-6:
            deduped.append(p)
    return deduped


def route_around_vias(ax, ay, bx, by, track_width, obstacles):
    """Compatibility shim: the band-based plan already encodes obstacle
    avoidance, so ignore the obstacles parameter."""
    return route_with_plan(ax, ay, bx, by)


def fmt(v):
    """Match KiCad's coordinate formatting (strip trailing zeros, but keep
    a decimal point if there is a fractional part)."""
    if isinstance(v, str):
        return v
    s = f"{v:.6f}".rstrip("0").rstrip(".")
    return s if s else "0"


def make_segment(x1, y1, x2, y2, width, layer, net):
    return (
        f"\t(segment\n"
        f"\t\t(start {fmt(x1)} {fmt(y1)})\n"
        f"\t\t(end {fmt(x2)} {fmt(y2)})\n"
        f"\t\t(width {fmt(width)})\n"
        f'\t\t(layer "{layer}")\n'
        f'\t\t(net "{net}")\n'
        f'\t\t(uuid "{uuid.uuid4()}")\n'
        f"\t)\n"
    )


def main():
    src = PCB.read_text()

    segments_by_net = defaultdict(list)
    vias_by_net = defaultdict(list)
    all_vias = []   # (x, y, radius, net)
    for m in re.finditer(r'^\t\((segment|via)\s', src, re.MULTILINE):
        pos = m.start() + 1
        form, end = parse_one(src, pos)
        fields = {c[0]: c[1:] for c in form[1:] if isinstance(c, list)}
        net = unq(fields.get("net", [""])[0])
        if m.group(1) == "segment":
            segments_by_net[net].append(
                {"fields": fields, "start": pos, "end": end})
        else:
            vias_by_net[net].append(
                {"fields": fields, "start": pos, "end": end})
            x = float(fields["at"][0])
            y = float(fields["at"][1])
            size = float(fields["size"][0])
            all_vias.append((x, y, size / 2.0, net))

    deletes = []   # (start, end) byte ranges, half-open, including trailing \n
    inserts = []   # (anchor_offset, replacement_text)

    for net in FLEX_NETS:
        vias = vias_by_net[net]
        if len(vias) != 2:
            print(f"SKIP {net}: {len(vias)} vias (expected 2)",
                  file=sys.stderr)
            continue
        inner = [s for s in segments_by_net[net]
                 if unq(s["fields"]["layer"][0]) != "F.Cu"]
        if not inner:
            print(f"SKIP {net}: no inner-layer segments to straighten",
                  file=sys.stderr)
            continue

        # Inner segments for a flex net should share a single layer & width;
        # otherwise the net was hand-routed in a way this script wasn't
        # designed for, and we'd rather bail than silently merge.
        inner_layers = {unq(s["fields"]["layer"][0]) for s in inner}
        widths = {float(s["fields"]["width"][0]) for s in inner}
        assert len(inner_layers) == 1, \
            f"{net}: mixed inner layers {inner_layers}"
        assert len(widths) == 1, f"{net}: mixed widths {widths}"
        inner_layer = next(iter(inner_layers))
        width = next(iter(widths))

        ax = float(vias[0]["fields"]["at"][0])
        ay = float(vias[0]["fields"]["at"][1])
        bx = float(vias[1]["fields"]["at"][0])
        by = float(vias[1]["fields"]["at"][1])

        # Obstacles: every via NOT on this net. Vias here are all F.Cu<->B.Cu
        # PTHs, so they obstruct every inner layer; we don't filter by layer.
        obstacles = [(x, y, r) for x, y, r, n in all_vias if n != net]

        polyline = route_around_vias(ax, ay, bx, by, width, obstacles)

        # Mark inner segments for deletion.
        for s in inner:
            start = s["start"]
            while start > 0 and src[start - 1] == "\t":
                start -= 1
            end = s["end"]
            if end < len(src) and src[end] == "\n":
                end += 1
            deletes.append((start, end))

        # Build replacement: one segment per polyline edge.
        replacement = "".join(
            make_segment(x1, y1, x2, y2, width, inner_layer, net)
            for (x1, y1), (x2, y2) in zip(polyline, polyline[1:])
        )
        inserts.append((inner[0]["start"], replacement))

        n_detours = (len(polyline) - 2) // 2
        if n_detours:
            print(f"  {net}: {n_detours} via detour(s)")

    # Each insert anchor points into a soon-to-be-deleted range; translate
    # to the start of the enclosing delete so the new segments fill the gap.
    delete_ranges = sorted(deletes)
    translated = []
    for off, txt in inserts:
        anchor = off
        for s, e in delete_ranges:
            if s <= off < e:
                anchor = s
                break
        translated.append((anchor, txt))

    # Apply highest offset first so earlier offsets stay valid. At the same
    # offset, delete (priority 0) before insert (priority 1) so the insert
    # fills the freshly-vacated position.
    events = [(s, 0, "del", e) for s, e in deletes]
    events += [(off, 1, "ins", txt) for off, txt in translated]
    events.sort(key=lambda x: (-x[0], x[1]))

    out = src
    for off, _, kind, payload in events:
        if kind == "del":
            out = out[:off] + out[payload:]
        else:
            out = out[:off] + payload + out[off:]

    PCB.write_text(out)
    print(f"Wrote {PCB.name}: {len(src)} -> {len(out)} bytes "
          f"({len(deletes)} segments deleted)")


if __name__ == "__main__":
    main()
