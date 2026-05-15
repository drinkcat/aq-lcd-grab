# aq-lcd-grab firmware

Embassy firmware for the Raspberry Pi Pico 2 W (RP2350) — capture target for
the [target device display protocol reverse-engineering](../docs/display_notes.md).

Currently just a hello world: blinks GPIO 25 and prints `hello from pico 2 w
— tick N` over USB CDC serial.

## Toolchain setup

```sh
rustup target add thumbv8m.main-none-eabihf
```

### Flashing tool: install `elf2uf2-rs` from git, not crates.io

The crates.io release (v2.2.0) **silently produces RP2040 UF2 files**
(family id `0xe48bff56`). The RP2350 boot ROM rejects them with no error —
the Pico just stays in BOOTSEL mode and you'll wonder why nothing runs.

The git version has a `deploy` subcommand and `--family rp2350-arm-s` flag:

```sh
cargo install --git https://github.com/JoNil/elf2uf2-rs elf2uf2-rs --force
```

The cargo runner in [.cargo/config.toml](.cargo/config.toml) uses the new
CLI form:

```toml
runner = "elf2uf2-rs deploy --family rp2350-arm-s"
```

## Flashing

1. Hold **BOOTSEL** while plugging in the Pico (or short RUN to GND).
2. The Pico enumerates as `2e8a:000f` and exposes a USB mass-storage drive,
   typically `/dev/sda1` on this laptop. It is **not** auto-mounted — do it
   manually:

   ```sh
   udisksctl mount -b /dev/sda1
   ```

3. Build and flash:

   ```sh
   cargo run --release
   ```

   `elf2uf2-rs` finds the mounted `RP2350` drive, converts the ELF to UF2
   with the correct family ID, and copies it over. The Pico reboots into
   the firmware automatically.

## Reading the USB serial output

After flashing, `/dev/ttyACM0` appears (the embassy-usb-logger CDC device):

```sh
cat /dev/ttyACM0
```

You should see `hello from pico 2 w — tick N` once per 500 ms.

## Pico 2 W gotcha: GPIO 25 is not the onboard LED

On the original Pico, GPIO 25 drives the onboard LED. On the **W** variant
(and Pico 2 W), the user LED is wired through the CYW43 wifi chip and needs
its firmware blobs loaded before it can be driven. The blink in this
firmware toggles GPIO 25 but you won't see it on the onboard LED — wire an
external LED if you want a visible blink. Trust the USB serial output as
the "is the firmware running" signal.
