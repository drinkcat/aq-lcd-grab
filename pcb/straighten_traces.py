"""One-shot trace straightener for aq_lcd_grab.kicad_pcb.

For every flex pass-through net (J1 <-> J2), replace all inner-layer segments
with a single straight segment between the net's two F.Cu<->B.Cu vias. F.Cu
fanout stubs and the vias themselves are left untouched.

Useful after KiCad's interactive router has piled on length-tuning serpentines
or you've otherwise made the inner-layer routing wonky. Not idempotent in any
deep sense — re-running just leaves the already-single segment in place.

Run from anywhere:
    python3 pcb/straighten_traces.py
"""

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


def main():
    src = PCB.read_text()

    segments_by_net = defaultdict(list)
    vias_by_net = defaultdict(list)
    for m in re.finditer(r'^\t\((segment|via)\s', src, re.MULTILINE):
        pos = m.start() + 1
        form, end = parse_one(src, pos)
        fields = {c[0]: c[1:] for c in form[1:] if isinstance(c, list)}
        net = unq(fields.get("net", [""])[0])
        bucket = segments_by_net if m.group(1) == "segment" else vias_by_net
        bucket[net].append({"fields": fields, "start": pos, "end": end})

    deletes = []   # (start, end) byte ranges, half-open, including trailing \n
    inserts = []   # (anchor_offset, replacement_text)

    for net in FLEX_NETS:
        vias = vias_by_net[net]
        if len(vias) != 2:
            print(f"SKIP {net}: {len(vias)} vias (expected 2)", file=sys.stderr)
            continue
        inner = [s for s in segments_by_net[net]
                 if unq(s["fields"]["layer"][0]) != "F.Cu"]
        if not inner:
            print(f"SKIP {net}: no inner-layer segments to straighten",
                  file=sys.stderr)
            continue

        # All inner segments for a flex net share one inner layer and one
        # width; if not, the net has been hand-routed in a way this script
        # isn't designed for and we'd rather bail than silently merge.
        inner_layers = {unq(s["fields"]["layer"][0]) for s in inner}
        widths = {s["fields"]["width"][0] for s in inner}
        assert len(inner_layers) == 1, f"{net}: mixed inner layers {inner_layers}"
        assert len(widths) == 1, f"{net}: mixed widths {widths}"
        inner_layer = next(iter(inner_layers))
        width = next(iter(widths))

        ax, ay = vias[0]["fields"]["at"]
        bx, by = vias[1]["fields"]["at"]

        for s in inner:
            start = s["start"]
            while start > 0 and src[start - 1] == "\t":
                start -= 1
            end = s["end"]
            if end < len(src) and src[end] == "\n":
                end += 1
            deletes.append((start, end))

        new_seg = (
            f"\t(segment\n"
            f"\t\t(start {ax} {ay})\n"
            f"\t\t(end {bx} {by})\n"
            f"\t\t(width {width})\n"
            f'\t\t(layer "{inner_layer}")\n'
            f'\t\t(net "{net}")\n'
            f'\t\t(uuid "{uuid.uuid4()}")\n'
            f"\t)\n"
        )
        inserts.append((inner[0]["start"], new_seg))

    # Each insert anchor points into a soon-to-be-deleted range. Translate
    # the anchor to the delete's start so the new segment lands in the gap.
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
          f"({len(deletes)} segments deleted, {len(translated)} inserted)")


if __name__ == "__main__":
    main()
