# ESP32-C6 Gateway App — Design Plan

Status: **DRAFT — for review.** No code has been written yet. Edit freely; we
implement once you're happy.

## Context

Today the display-bus capture lands on a host PC: an STM32F103 (UART @ 921600)
or a Pico 2 W (USB CDC) sniffs the 8080 bus, RLE-encodes samples with the
`wire` crate, and ships them to the `host/` viewer. The host runs the whole
pipeline — `wire` decode → board `permute` → glyph `decoder` (sensor values)
and → `framebuffer` (RGB565 image reconstruction).

We want a standalone, always-on **ESP32-C6 gateway** that removes the PC from
the loop:

1. Receive the existing **wire-protocol byte stream over UART** from the
   STM32/Pico bridge (decided: ESP32 does *not* tap the bus directly in v1).
2. Run the full decode pipeline on-chip.
3. **Publish sensor values to Home Assistant over MQTT** (pm25, tvoc, co2,
   temp, humidity).
4. **Serve the reconstructed framebuffer as an image** over a small embedded
   HTTP server, for live viewing in a browser.

This reuses the `wire` + `decoder` + `framebuffer` logic that already works on
the host, and the WiFi + MQTT + secrets scaffolding proven in the sibling
`zappy-esp` project.

## Decisions (confirmed)

- **UART input** = existing wire protocol from STM32/Pico. ESP32 is a network
  gateway, not a bus sniffer.
- **Outputs (v1)** = MQTT sensor values to HA **and** an HTTP framebuffer image.
- **Decoder `flush()`** = replace the allocating `Vec<RowReport>` API with a
  **callback** `flush_each(&mut self, f: impl FnMut(&str name, &str value))`.
  The data is tiny and bounded (≤5 rows, ≤8 chars each), so the value is built
  into a small stack buffer and passed as a borrowed `&str` — **no allocator
  needed anywhere in the `decoder` crate** (it becomes pure `no_std`, no
  `alloc`). The host adapts its call sites to the callback (or a thin
  `std`-only wrapper that collects into a `Vec`).
- **Framebuffer** = 4 bits-per-pixel into a **fixed 16-entry RGB565 palette**
  (~75 KiB, not ~300 KiB full RGB565). Palette derived from a real capture.
- **HTTP server** = hand-rolled single-route `embassy-net::tcp` handler (no web
  framework), serving a palettized BMP.
- **TVOC unit** = ppm (matches the panel label).
- **New crate** = `firmware-esp32/`, standalone with path deps. **No root
  workspace** — keep the current per-crate convention.

## What already exists and is reusable

| Piece | Location | Reuse plan |
|---|---|---|
| Wire encoder + `Sink` | [wire/src/encoder.rs](../wire/src/encoder.rs), [wire/src/sink.rs](../wire/src/sink.rs) | Already `no_std`. Used as-is (only needed for host→fw commands; see below). |
| Wire **decoder** (`Decoder::feed` → `Event`s) | [host/src/wire.rs](../host/src/wire.rs) | **Port into the `wire` crate** as `no_std`. Currently host-only (`std::io`, `Vec`, `String`). ~200 lines, mechanical port. |
| Board permute (`permute_pico` / `permute_f103`) | [host/src/permute.rs](../host/src/permute.rs) | **Move into the `wire` crate** (pure, already `no_std`-compatible). |
| Glyph decoder (`Decoder::feed`/`flush`, `ROWS`, templates) | [decoder/src/lib.rs](../decoder/src/lib.rs) | Make `no_std` + `alloc`; add allocation-free flush. Templates baked by `build.rs` (no change). |
| Framebuffer (RGB565 replay, `apply`, `to_rgba8`) | [host/src/framebuffer.rs](../host/src/framebuffer.rs) | **Move into a shared `no_std` crate**; drop the egui-oriented `WindowWrite` dumping path; keep `apply` + an image exporter. |
| WiFi + embassy-net + MQTT + secrets/build.rs | [zappy-esp/src/bin/main.rs](../../zappy-esp/src/bin/main.rs), [zappy-esp/build.rs](../../zappy-esp/build.rs) | Copy patterns: `esp_radio::init`, `embassy_net::new`, `rust-mqtt` client + TCP shim, HA discovery, compile-time secrets from `secrets.env`. |

### Pipeline (host today → ESP32 target)

```
UART bytes ─► wire::Decoder ─► Event{Block|Run|Repeat2|Tick|Log|Overrun|...}
                                     │ (raw u32 samples)
                                     ▼
                              permute_f103 / permute_pico ─► (data:u16, is_data:bool)
                                     │
                        ┌────────────┴─────────────┐
                        ▼                           ▼
              glyph Decoder.feed             Framebuffer.apply
              (+ flush on idle/Log)          (RGB565 300 KB)
                        │                           │
                        ▼                           ▼
                MQTT publish to HA           HTTP server → PNG/BMP
```

## Crate restructuring (the only structural change)

The repo currently has standalone crates with path deps (no root workspace).
The host-only decode logic (`wire.rs` decoder, `permute.rs`, `framebuffer.rs`)
lives under `host/src/` and pulls in `std`. To share it with firmware we lift
the pure logic into `no_std` crates:

1. **`wire` crate gains a streaming, alloc-free decoder + permute:**
   - Add `wire/src/decoder.rs` ported from `host/src/wire.rs`, but **not** the
     `io::Result<Vec<Event>>` shape. The only variable-length events are
     `Block` (a list of samples) and `Repeat2` (a run-length list); rather than
     collect those into `Vec`s, the decoder **emits decoded samples one at a
     time through a callback**:
     ```
     fn feed(&mut self, bytes: &[u8], on: impl FnMut(WireOut)) -> Result<(), WireError>
     enum WireOut { Sample(u32), Tick{..}, Overrun{u32}, Log(&str), Started, Stopped }
     ```
     `Block`/`Run`/`Repeat2` all expand to a sequence of `WireOut::Sample(u32)`
     calls (RUN/REPEAT2 just repeat the same value N times) — exactly what the
     downstream consumers want, since both the glyph decoder and framebuffer
     take a `(data, is_data)` *stream*. `Log` borrows a `&str` from the internal
     buffer (valid for the callback's duration). **No `Vec`, no `String`, no
     `alloc`.**
   - Frame reassembly across reads needs a small byte buffer for a partial
     trailing frame. Use a fixed `heapless`-style `[u8; N]` (a frame is bounded
     by the firmware's TX staging; size N to the largest frame, e.g. a 256-entry
     BLOCK ≈ 1 KiB) and return `WireError::Overflow` if exceeded — still no heap.
   - Add `wire/src/permute.rs` (move `permute_pico`/`permute_f103` verbatim;
     already pure / `no_std`).
   - Update [host/src/main.rs](../host/src/main.rs) and friends to drive the
     callback decoder (the host can collect into a `Vec` in its own closure if
     convenient); delete `host/src/wire.rs` and `host/src/permute.rs`.
   - The `Event` enum (with `Vec`/`String`) is dropped, or kept host-only behind
     `feature = "std"` if the host viewer prefers batch events.

2. **`decoder` crate → pure `no_std` (no `alloc`):**
   - Add `#![no_std]`. The `Decoder` already holds only fixed arrays
     (`[RowState; ROWS_LEN]`, `[Option<Slot>; MAX_DIGITS_PER_ROW]`),
     `&'static str` labels, and baked template statics — nothing needs the heap.
   - **Replace** the allocating `flush() -> Vec<RowReport>` with a callback:
     `flush_each(&mut self, f: impl FnMut(&str /*name*/, &str /*value*/))`.
     The value is bounded (≤ `MAX_DIGITS_PER_ROW` = 8 ASCII chars), so build it
     into a small `[u8; 8]` stack buffer, `core::str::from_utf8(..).trim()`, and
     hand the borrowed `&str` to the callback. No `String`, no `Vec`, no
     `heapless` even.
   - `RowReport`/`String` go away (or move behind `feature = "std"` only if the
     host wants the old shape). Host call sites switch to `flush_each` — e.g. a
     `|name, val| values.insert(name, val.to_string())` closure on the host.
   - `build.rs` / templates unchanged.

3. **New shared `framebuffer` crate** (pure `no_std`, no `alloc`), moved from
   `host/src/framebuffer.rs`:
   - The 75 KiB pixel buffer is **not** a `Vec`. The crate operates on a
     caller-provided slice: `Framebuffer::new(buf: &'static mut [u8; 76800])`
     (or a `&mut [u8]` with a length assert). On the ESP32 the buffer comes from
     a `StaticCell`; on the host it's a `Box`/`Vec` the host owns and lends.
     No allocator in the crate.
   - Keep `Framebuffer::new/apply` (replay 0x2A/0x2B/0x2C/0x3C), but store
     pixels as **4 bits-per-pixel into a fixed 16-entry RGB565 palette**
     instead of one `u16` per pixel. This shrinks the buffer from
     320×480×2 = 307,200 B (~300 KiB) to 320×480×0.5 = **76,800 B (~75 KiB)**
     plus a 16×2 = 32 B palette — a 4× reduction that fits comfortably
     alongside WiFi/MQTT/HTTP buffers.
   - **Fixed palette:** the panel is a UI with a small known set of colors
     (panel-background reds/greens, white/black text, a few unit-label grays),
     so a static palette baked into the crate is sufficient. Map each incoming
     RGB565 pixel → palette index: exact-match the known colors first, fall
     back to nearest-by-component distance for stragglers. The palette is
     derived from a real capture during implementation (see step below); it is
     **not** built dynamically at runtime.
   - The host viewer keeps full fidelity if desired by using the identity/large
     palette under `feature = "std"`, or simply renders via the same 4bpp path
     (the on-screen panel has few enough colors that 16 is lossless in
     practice — confirm when the palette is derived).
   - Replace the `Frame`-based `apply` input with the same `(data, is_data)`
     stream the glyph decoder consumes, so both decoders share one feed loop
     (the glyph decoder already does its own 8080 framing — the framebuffer
     needs an equivalent tiny command tracker, or keep feeding it via the
     existing `bus_decoder::Frame` path on the host and a thin inline tracker
     on the ESP32). **Recommended:** give `Framebuffer` a `feed(data, is_data)`
     method mirroring the decoder's command framing, so the host's
     `bus_decoder` becomes host-only/optional.
   - Add an alloc-free image exporter usable on-device. A **4bpp palettized
     BMP** is a perfect fit — BMP natively supports an indexed color table, so
     the on-wire image stays palettized (no RGB888 expansion on the ESP32, the
     browser does the lookup) and the pixel data is *already* the framebuffer's
     4bpp bytes. Emit it as a small fixed BMP header + palette table written to
     the TCP socket, then the 75 KiB pixel slice streamed directly — `to_bmp`
     either writes into a caller buffer or returns an `impl Iterator<Item=u8>`
     (header/palette chained with `self.buf.iter()`), zero allocation. Keep the
     `std`-only `to_rgba8` for the egui host viewer.

## New firmware crate: `firmware-esp32/`

A new standalone crate modeled on `zappy-esp`. Key files:

- `Cargo.toml` — deps mirrored from zappy-esp:
  - `esp-hal ~1.0` (`esp32c6`, `unstable`, `log-04`),
    `esp-rtos` (`embassy`, `esp-alloc`, `esp-radio`),
    `esp-radio` (`wifi`, `smoltcp`), `embassy-net` (`dhcpv4`, `dns`, `tcp`),
    `embassy-executor`, `embassy-time`, `embassy-sync`, `static_cell`,
    `esp-alloc`, `esp-backtrace`, `esp-println`.
  - `rust-mqtt` (`alloc`, `v5`) + `embedded-io-async` shim (HA path).
  - **Local deps:** `wire`, `decoder`, `framebuffer` (path deps, `no_std`).
  - For HTTP server: `picoserve` (embassy-native async HTTP) **or** a hand-rolled
    `embassy-net::tcp` handler (the image response is a single fixed route, so
    a minimal hand-rolled handler is viable and dependency-light). *Open
    question below.*
- `.cargo/config.toml` — `riscv32imac-unknown-none-elf`, runner
  `espflash flash --monitor --chip esp32c6 -B 921600`.
- `rust-toolchain.toml` — stable + `rust-src` + the riscv target.
- `build.rs` — load `secrets.env` (`WIFI_SSID`, `WIFI_PASSWORD`, `HA_HOST`,
  `HA_USER`, `HA_TOKEN`), `esp_app_desc!()`, linker niceties (copy from zappy).
- `secrets.env` (gitignored).

### Tasks / architecture

```
main():
  init clocks (CpuClock::max), esp_alloc heap (~96 KB; framebuffer is static, not heap)
  esp_rtos::start(timg0, sw_int)
  WiFi: esp_radio::init → wifi::new → ClientConfig(ssid,pw) → start
  embassy_net::new(dhcpv4) → spawn net_task
  spawn wifi_task        (connect/reconnect loop — from zappy)
  spawn uart_task        (NEW: drive the decode pipeline)
  spawn mqtt_task        (publish values; HA discovery — adapted from zappy)
  spawn http_task        (NEW: serve framebuffer image)
```

Shared state:
- `static FB: Mutex<RawMutex, Framebuffer>` (or a double-buffer / `&'static`
  via `StaticCell`) — written by `uart_task`, read by `http_task`. With the 4bpp
  palettized buffer this is ~75 KiB in static RAM (vs ~300 KiB for full RGB565),
  leaving ample room for WiFi/MQTT/HTTP buffers on the C6's 512 KiB SRAM.
- `PubSubChannel<RawMutex, RowUpdate, ...>` — `uart_task` publishes
  `(name, value)` updates; `mqtt_task` subscribes (mirrors zappy's `ZAP_PUBSUB`).

`uart_task` (the core new loop):
- Configure UART (decided params below) with async RX + DMA.
- One `wire::Decoder`, one `decoder::Decoder`, the shared `Framebuffer`.
- Read bytes → `wire::Decoder` callback → for each `Event`:
  - `Block`/`Run`/`Repeat2` → expand to samples → `permute_f103`/`permute_pico`
    → for each `(data, is_data)`: `glyph.feed(...)` **and** `fb.feed(...)`.
  - `Log` event or RX idle timeout → `glyph.flush_each(|name, val| publish)`.
  - `Tick`/`Overrun`/`Started`/`Stopped` → log / health only.
- On boot, perform the host's **sync handshake** over UART TX: send `STOP`,
  drain, send `START`, wait for `STARTED` (reuse `wire::HOST_CMD_*`).

`mqtt_task` (adapted from zappy `mqtt_task`):
- Connect to `HA_HOST:1883` with `HA_USER`/`HA_TOKEN`.
- Publish MQTT discovery configs for 5 sensors with correct
  `device_class`/`unit_of_measurement`:
  - pm25 → `pm25`, µg/m³; co2 → `carbon_dioxide`, ppm;
    tvoc → `volatile_organic_compounds_parts`, ppm (the panel labels it ppm);
    temp → `temperature`, °C; humidity → `humidity`, %.
  - One HA `device` (`identifiers:["aq-lcd"]`) grouping all five.
- Subscribe to the values pubsub; publish `state_topic` on change + a keepalive.

`http_task` (NEW) — **hand-rolled single-route `embassy-net::tcp` handler**
(decided; no `picoserve`/web framework, since it's one image + one HTML page):
- Bind TCP :80, accept a connection, read the request line, branch on path.
- On `GET /` return a tiny HTML page that `<img src="/fb.bmp">` with a
  meta-refresh (or JS poll).
- On `GET /fb.bmp`: lock `FB` and stream a **palettized BMP** (4bpp/8bpp indexed
  with the 16-entry color table; 180° rotation as in `to_rgba8`). Stays
  palettized on the wire — no RGB888 expansion on-device, no PNG dependency.

### Pins / UART params (decided defaults; confirm against the bridge wiring)

- UART1 RX from the STM32 TX, **921600 8N1** (matches `firmware-stm32`).
  Pins: pick two free GPIOs on the C6 (e.g. GPIO16=RX, GPIO17=TX) — final pins
  to match the physical board. TX wired back to the bridge for START/STOP.
- The `permute_*` choice depends on which bridge feeds the ESP32 — default
  `permute_f103` (STM32 over UART). Make it a build feature
  (`bridge-stm32` / `bridge-pico`), defaulting to STM32.

## Verification

1. **Host regression first:** after lifting `wire`/`permute`/`framebuffer`
   into shared `no_std` crates, `cargo test` in `decoder/` and `wire/`, and
   `cargo run` the `host/` viewer against a recorded capture
   (`reference/goodrun/run2.bin`) — values and image must be unchanged. This
   de-risks the refactor before any ESP32 code.
2. **Decoder unit tests** (existing `decoder/src/lib.rs` tests + new
   `flush_each` test) pass under both `std` and a `no_std`+`alloc` build
   (`cargo build --no-default-features` with a `target` like
   `riscv32imac-unknown-none-elf` or `thumbv7em` to prove no_std compiles).
3. **ESP32 bring-up, incrementally:**
   - a. WiFi associates, gets DHCP (serial log).
   - b. UART pipeline: feed it from the real STM32/Pico bridge (or replay
     `run2.bin` out a USB-UART into the C6's RX); confirm logged decoded values
     match the host.
   - c. HTTP: browse `http://<esp-ip>/` and see the reconstructed panel image
     updating (validates capture→permute→framebuffer end-to-end).
   - d. MQTT: sensors appear in Home Assistant, values update live.
4. Flash/run: `cargo run --release` from the new crate (espflash + monitor).

## Open questions / risks

1. **RAM budget:** the 4bpp palettized framebuffer is ~75 KiB (down from
   ~300 KiB full RGB565), so it comfortably coexists with WiFi/MQTT/HTTP
   buffers on the C6's 512 KiB SRAM (note some SRAM is reserved for
   ROM/bootloader and the radio). Keep TLS off (HA MQTT on plaintext 1883) and
   lock-and-stream the image to avoid a second copy. Further headroom available
   via downscale if ever needed, but not expected.
   - **Confirm the panel really uses ≤16 colors** when deriving the palette; if
     anti-aliasing/gradients push it higher, either widen to 8bpp (256 colors,
     150 KiB — still fine) or snap near-colors to the nearest of 16.
2. **MQTT auth:** zappy passes `HA_USER`/`HA_TOKEN`; confirm the HA broker
   credentials model (Mosquitto user vs HA long-lived token).

## Suggested implementation order

1. Lift the `wire` decoder + `permute` into the `wire` crate as a streaming,
   alloc-free callback decoder; update host; green host tests + viewer.
   *(no ESP32 yet)*
2. Make `decoder` pure `no_std` (no `alloc`); replace `flush()` with the
   `flush_each` callback; update host call sites; prove no_std build.
3. Extract `framebuffer` crate (pure `no_std`, caller-provided buffer); add
   `feed(data,is_data)`, 4bpp + fixed 16-entry RGB565 palette storage, and an
   alloc-free palettized BMP exporter; update host viewer to use it. **Derive
   the palette first:** instrument the host to dump distinct RGB565 values when
   replaying `reference/goodrun/run2.bin`, confirm there are ≤16 (or pick 8bpp),
   and bake that palette into the crate.
4. Scaffold `firmware-esp32/` from zappy (build/flash/secrets/WiFi/net only).
5. `uart_task` + pipeline; verify decoded values in serial log.
6. `http_task` → framebuffer image (do this before MQTT — it validates the whole
   capture→permute→framebuffer path visually, which is the best end-to-end sanity
   check before wiring up Home Assistant).
7. `mqtt_task` → Home Assistant.
