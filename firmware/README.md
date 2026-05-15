# aq-lcd-grab firmware

Embassy firmware for the Raspberry Pi Pico 2 W (RP2350) — capture target for
the [target device display protocol reverse-engineering](../docs/display_notes.md).

Current state: PIO+DMA capture PoC. Waits for write strobes on GPIO 18 (WR),
samples `{CS, DC, DB15..DB0}` on each WR rising edge, drains a 4096-word
buffer via DMA, and dumps the first 32 decoded samples over USB CDC. With
nothing connected to GPIO 18, the capture loop suspends correctly — proving
the PIO/DMA wiring is sound.

## Toolchain setup

```sh
rustup target add thumbv8m.main-none-eabihf
sudo pacman -S picotool        # /usr/bin/picotool, v2.2.0+
```

## Flashing

`picotool` does flash+verify+execute in one shot. The cargo runner is set
to `picotool load -u -v -x -t elf` in [.cargo/config.toml](.cargo/config.toml).

1. Hold **BOOTSEL** while plugging in the Pico (or short RUN to GND). The
   device enumerates as `2e8a:000f` (Raspberry Pi RP2350 Boot).
2. Build and flash:

   ```sh
   cargo run --release
   ```

   No mass-storage mount needed. picotool talks to the BOOTSEL ROM over
   USB directly, writes the ELF to flash, verifies, and executes.

## Reading the USB serial output

After flashing, `/dev/ttyACM0` appears (the embassy-usb-logger CDC device):

```sh
stty -F /dev/ttyACM0 raw -echo
cat /dev/ttyACM0
```

**Start the reader before reboot** — otherwise the startup log lines are
lost. Run `cat /dev/ttyACM0` in a separate terminal, then `cargo run` in
another.

You should see:

```
aq-lcd-grab capture PoC starting
waiting for 4096 samples on WR (GPIO 18)…
```

…and then nothing until WR pulses arrive on GPIO 18.

## Pin assignment

| Pico GPIO | Function |
|-----------|----------|
| 0–15 | DB0–DB15 (16-bit data bus, must be consecutive for PIO `in pins, 16`) |
| 16 | D/C (RS) |
| 17 | CS |
| 18 | WR (write strobe — sample trigger) |
| 22, 26–28 | spare |
| 23, 24, 25, 29 | reserved for CYW43 wifi |

## Notes

### Earlier flashing approach (deprecated)

We previously used `elf2uf2-rs` from git (the crates.io v2.2.0 silently
produced RP2040 UF2s that the RP2350 boot ROM rejected). `picotool` makes
that whole detour irrelevant — it speaks the BOOTSEL protocol directly and
handles family IDs internally.

### Pico 2 W onboard LED

GPIO 25 drives the onboard LED on the original Pico, but on the W variant
the user LED is wired through the CYW43 wifi chip (firmware-blob-loaded).
Use USB CDC output as the "firmware alive" signal.

### Watch your serial reader

If `cat /dev/ttyACM0` shows nothing, it's almost always because the host
opened the tty after the firmware already printed its startup messages —
the kernel doesn't buffer pre-open output. Start the reader first, then
power-cycle / re-flash the Pico.
