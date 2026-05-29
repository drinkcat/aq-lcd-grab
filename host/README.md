# aq-lcd-grab viewer

Live decoder for the capture board's tagged wire protocol (see
[`docs/wire_protocol.md`](../docs/wire_protocol.md)). Opens the capture
board's serial device, runs the START/STOP handshake, parses the
incoming sample stream into 8080 bus transactions, replays them into a
framebuffer, and renders both the framebuffer and a live activity log.

```sh
cargo run --release -- --port /dev/ttyACM0
```

Add `--dump-dir PATH` to dump every detected glyph window as a PNG;
`--replay FILE` to feed a previously captured raw byte stream instead
of opening the serial port.

## Diagnostics

### Tell the firmware to dump its counters

The firmware accepts a `STATS` (`0x04`) command and replies with a log
line containing `tx_dropped` (TX-pipe bytes the encoder couldn't fit
on the wire) and `cap_dropped` (PIO/DMA ring samples lost on the
capture side). The viewer doesn't have a UI hook for this yet — pop
the byte in manually with `printf '\x04' > /dev/ttyACM0` while the
viewer is running and watch the log panel.

### Inspect the wire with usbmon (Pico)

When pixels look wrong it's useful to know whether bytes are being
lost on the USB link, by the host parser, or never produced. usbmon
captures every URB at the kernel level so you can compare against
what the firmware says it sent and what the viewer says it parsed.

```sh
# Find the Pico's bus number.
lsusb -d c0de:cafe                       # e.g. "Bus 003 Device 090: ..."

# In one terminal — capture all USB traffic on that bus.
sudo modprobe usbmon                     # if not already loaded
sudo tcpdump -i usbmon3 -w /tmp/pico.pcap

# In another — reproduce whatever was misbehaving.
cargo run --release -- --port /dev/ttyACM0 > /tmp/viewer.out 2>&1

# Ctrl-C tcpdump. Then summarise the bulk-IN endpoint (EP 0x82):
tshark -r /tmp/pico.pcap \
  -Y 'usb.endpoint_address.number == 2
      and usb.endpoint_address.direction == 1
      and usb.transfer_type == 0x03
      and usb.data_len > 0' \
  -T fields -e frame.time_relative -e usb.data_len \
  | awk '{n++; b+=$2} END {printf "%d URBs, %d bytes, avg %.1f B/URB\n", n, b, b/n}'
```

Notes:
- `usbmon3` = bus 3; pick the bus your Pico is on.
- usbmon records every URB twice (Submit + Complete). The Submit
  entries carry `data_len=0` for IN transfers, so always filter on
  `usb.data_len > 0` or you'll double-count ZLPs you don't have.
- Wire-rate bursts of ~200 kB/s happen briefly during display screen
  redraws — that's normal and well within USB FS bulk. Sustained
  rates above ~60 kB/s are the danger zone.

Useful per-time-window histogram (100 ms bins) to spot bursts:

```sh
tshark -r /tmp/pico.pcap \
  -Y 'usb.endpoint_address.number == 2
      and usb.endpoint_address.direction == 1
      and usb.transfer_type == 0x03
      and usb.data_len > 0' \
  -T fields -e frame.time_relative -e usb.data_len \
  | awk 'BEGIN { bin=0.1 }
         { bytes[int($1/bin)] += $2; if (int($1/bin) > m) m = int($1/bin) }
         END { for (i=0; i<=m; i++) printf "%5.1f  %d\n", i*bin, bytes[i]+0 }'
```

Cross-check: bytes-on-the-wire (from tshark) should match the
viewer's parsed byte count exactly. If usbmon shows more bytes than
the viewer parsed, the host parser is at fault; if usbmon shows
fewer than the firmware claims it sent (compare against `tx_dropped`
from STATS — anything dropped never reaches usbmon), suspect the
USB driver or `commit_frame`. The `tag=0xFD` overrun frames that
appear in the activity log mark every gap the firmware noticed.
