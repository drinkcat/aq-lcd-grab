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
DETOUR_PAD = 0.005  # mm, extra slack on the perpendicular offset magnitude
JOG_LEAD_IN = 0.05  # mm, extra longitudinal slack so the diagonal corner
                    # sits clearly outside the obstacle's clearance circle —
                    # "turn earlier" margin

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


# Flex vias sit on a staggered 4-band grid per connector side.
#   J2 (top) bands:    33.24, 36.04, 38.94, 40.94
#   J1 (bottom) bands: 68.69, 65.89, 62.99, 60.99
# Pairing (top↔bottom) for each net depends on which J1 pin number it's
# wired to:
#   33.24 ↔ 65.89    36.04 ↔ 68.69    38.94 ↔ 62.99    40.94 ↔ 60.99
#
# The "deep" bands (33.24 / 36.04 on top, 68.69 / 65.89 on bottom) are on
# In2.Cu; their traces must thread past the "shallow" bands' via columns
# (38.94, 40.94 on top; 62.99, 60.99 on bottom). The shallow-band nets
# themselves are on In1.Cu and route straight through.
#
# A deep-band trace passes two obstacle rows per side. At each row, the
# nearest same-layer obstacle column sits Δx ≈ ±0.30 mm from the trace's
# column. The trace dodges to the opposite side, with offset just enough
# to clear:
#
#     offset = (orad + CLEARANCE + half_track + DETOUR_PAD) − |Δx_obstacle|
#
# Jogs are 45° diagonals (|Δx| == |Δy|). The longitudinal jog lands the
# trace at the target offset by the time y reaches `obstacle_y − offset`,
# so the diagonal's corner sits at `obstacle_y − 2*offset` (entering) and
# `obstacle_y + 2*offset` (exiting).

# Shallow-band y-values per side (the via rows we need to dodge).
SHALLOW_J2 = (38.94, 40.94)
SHALLOW_J1 = (62.99, 60.99)
DEEP_J2 = (33.24, 36.04)
DEEP_J1 = (68.69, 65.89)


def band_for(y, choices):
    """Return the band value (one of `choices`) that y is closest to within
    0.05 mm tolerance, or None."""
    for b in choices:
        if abs(y - b) < 0.05:
            return b
    return None


def nearest_obstacle_dx(col_x, obstacle_row_y, obstacles):
    """Among `obstacles` (list of (ox, oy, orad)) sitting on `obstacle_row_y`,
    return the signed Δx (ox − col_x) of the column closest to `col_x`.
    Returns None if no obstacle on that row."""
    best = None
    for ox, oy, orad in obstacles:
        if abs(oy - obstacle_row_y) > 0.05:
            continue
        dx = ox - col_x
        if best is None or abs(dx) < abs(best[0]):
            best = (dx, orad)
    return best   # (dx, orad) or None


def build_fanout(col_x, via_y, sign, obstacle_rows, obstacles, half_w):
    """Build the fanout waypoint list for one side of the trace, from the
    via outward toward the middle of the board.

    Args:
        col_x: trace column x position.
        via_y: starting via y position.
        sign:  +1 if traveling toward larger y (J2/top fanout going down),
               −1 if traveling toward smaller y (J1/bottom fanout going up).
        obstacle_rows: list of y-values of the obstacle rows (in order
            of distance from via — nearest first).
        obstacles: list of (ox, oy, orad) for the layer.
        half_w: half of the track width.

    Returns: list of (x, y) waypoints starting at the via and ending at the
        first "on-column" point past the last obstacle.
    """
    pts = [(col_x, via_y)]
    cur_offset = 0.0
    for row_idx, row_y in enumerate(obstacle_rows):
        info = nearest_obstacle_dx(col_x, row_y, obstacles)
        if info is None:
            continue
        dx_obs, orad = info
        needed = orad + CLEARANCE + half_w + DETOUR_PAD
        if abs(dx_obs) >= needed:
            # Already clear at the column; no jog needed for this row.
            continue
        # Target offset on the side opposite the obstacle.
        offset_mag = needed - abs(dx_obs)
        target_offset = -math.copysign(offset_mag, dx_obs)
        # Turn earlier than strictly geometrically required so the diagonal
        # corner has slack from the obstacle. `offset_mag` is the minimum;
        # add JOG_LEAD_IN for extra margin matching the hand-routed pattern.
        delta = target_offset - cur_offset
        jog_arrival_y = row_y - sign * (offset_mag + JOG_LEAD_IN)
        jog_start_y = jog_arrival_y - sign * abs(delta)
        # Emit the jog start (on column at cur_offset) and the jog arrival.
        pts.append((col_x + cur_offset, jog_start_y))
        pts.append((col_x + target_offset, jog_arrival_y))
        cur_offset = target_offset
    # Final return to column past the last obstacle (symmetric to entry).
    if abs(cur_offset) > 1e-9 and obstacle_rows:
        last_row_y = obstacle_rows[-1]
        offset_mag = abs(cur_offset)
        hold_end_y = last_row_y + sign * (offset_mag + JOG_LEAD_IN)
        return_y = hold_end_y + sign * offset_mag
        pts.append((col_x + cur_offset, hold_end_y))
        pts.append((col_x, return_y))
    return pts


def route_with_plan(ax, ay, bx, by, track_width, obstacles):
    """Polyline from A to B with 45° jogs around each shallow-band obstacle.
    Deep-band nets get jogs computed from their column's position relative
    to the actual obstacle columns; shallow-band nets go straight through.
    """
    if ay < by:
        top_x, top_y, bot_x, bot_y = ax, ay, bx, by
    else:
        top_x, top_y, bot_x, bot_y = bx, by, ax, ay

    half_w = track_width / 2.0
    top_deep = band_for(top_y, DEEP_J2) is not None
    bot_deep = band_for(bot_y, DEEP_J1) is not None
    if not (top_deep or bot_deep):
        return [(ax, ay), (bx, by)]

    # The trace's column x is the average of the two via x's (they're
    # nearly identical for flex pass-through).
    col_x = (top_x + bot_x) / 2

    # Top fanout (going from top via DOWN, sign=+1). Obstacle rows = J2
    # shallow rows, sorted by distance from top via.
    top_rows = sorted(SHALLOW_J2, key=lambda r: abs(r - top_y)) if top_deep else []
    top_pts = build_fanout(col_x, top_y, +1, top_rows, obstacles, half_w)
    # Re-anchor first point to the actual top via x (might differ slightly
    # from col_x due to the 0.02 mm stagger between top/bot via positions).
    top_pts[0] = (top_x, top_y)

    # Bottom fanout (going from bot via UP, sign=−1).
    bot_rows = sorted(SHALLOW_J1, key=lambda r: abs(r - bot_y)) if bot_deep else []
    bot_pts = build_fanout(col_x, bot_y, -1, bot_rows, obstacles, half_w)
    bot_pts[0] = (bot_x, bot_y)
    # The bottom fanout starts at the bot via and goes toward the middle;
    # we need to reverse it so we can append after the top fanout.
    bot_pts.reverse()

    pts = top_pts + bot_pts

    deduped = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - deduped[-1][0], p[1] - deduped[-1][1]) > 1e-6:
            deduped.append(p)
    return deduped


def route_around_vias(ax, ay, bx, by, track_width, obstacles):
    """Entry point used by the main rewriter."""
    return route_with_plan(ax, ay, bx, by, track_width, obstacles)


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
