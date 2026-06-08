# aq-lcd-grab ESP32-C6 gateway

Standalone ESP32-C6 firmware (esp-hal + Embassy) that:

1. receives the wire-protocol capture stream over UART from the STM32/Pico
   bridge,
2. decodes it on-chip (`wire` → `permute` → glyph `decoder` + `framebuffer`),
3. publishes sensor values to Home Assistant over MQTT, and
4. serves the reconstructed panel image over HTTP.

Reuses the shared `no_std` crates (`../wire`, `../decoder`, `../framebuffer`)
that the host viewer uses, so the decode logic can't drift.

## Status

Bring-up scaffold: WiFi + DHCP. UART/decode, HTTP, and MQTT tasks land in
subsequent steps (see `../docs/esp32_app_plan.md`).

## Setup

```sh
cp secrets.env.example secrets.env
# edit secrets.env: WIFI_SSID, WIFI_PASSWORD, HA_HOST, HA_USER, HA_TOKEN
```

`secrets.env` is gitignored; `build.rs` bakes the values in at compile time.

## Build / flash

```sh
cargo build --release        # build only
cargo run --release          # flash + monitor (espflash, 921600 baud)
```

Target/runner are configured in `.cargo/config.toml`
(`riscv32imac-unknown-none-elf`, `espflash flash --monitor --chip esp32c6`).

## Features

- `bridge-stm32` (default): UART input is permuted with `permute_f103`
  (STM32F103 bench rig, 921600 8N1).
- `bridge-pico`: UART input is permuted with `permute_pico`.
- `homeassistant` (default): publish decoded values via MQTT.
