#!/usr/bin/env bash
# Cargo runner for the aq-lcd-grab firmware.
# Flashes, verifies and executes the ELF binary on a Pico 2 W via picotool.
#
# Three cases:
#   1. Pico in BOOTSEL (VID:PID 2e8a:000f) → picotool finds it directly.
#   2. Pico running aq-lcd-grab (c0de:cafe) → ask the firmware to reboot
#      into BOOTSEL via its picotool reset interface, then flash. picotool
#      tries to do this in one shot with `-f`, but its serial-tracking
#      misbehaves on our firmware (embassy-usb leaks garbage into the
#      serial-number descriptor), so we drive the two steps manually.
#   3. Nothing found → tell the user to plug in / hold BOOTSEL.
#
# picotool quirk: device-selection flags must come AFTER the input file.

set -euo pipefail

ELF="$1"

wait_for_bootsel() {
    for _ in $(seq 1 30); do
        if lsusb | grep -q "2e8a:000f"; then
            return 0
        fi
        sleep 0.5
    done
    return 1
}

if lsusb | grep -q "2e8a:000f"; then
    exec picotool load -u -v -x -t elf "$ELF"
elif lsusb | grep -q "c0de:cafe"; then
    echo "flash.sh: asking running firmware to reboot into BOOTSEL…"
    # Best-effort reboot — picotool may exit with an error due to bogus
    # serial tracking, but the device usually reboots anyway.
    picotool reboot --vid 0xc0de --pid 0xcafe -f -u || true
    if wait_for_bootsel; then
        exec picotool load -u -v -x -t elf "$ELF"
    else
        echo "flash.sh: device didn't appear in BOOTSEL after reboot." >&2
        exit 1
    fi
else
    echo "flash.sh: no Pico found (neither 2e8a:000f nor c0de:cafe)." >&2
    echo "  Plug it in, or hold BOOTSEL and replug if the firmware is wedged." >&2
    exit 1
fi
