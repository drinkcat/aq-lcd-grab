# `host/` design

The viewer turns a serial byte stream from a capture board into a live
reconstruction of the capture target's display, plus a side panel of decoded
sensor values.

## End-to-end pipeline

```
  capture board                              host viewer
  ─────────────                              ───────────
  WR strobe + GPIO   ──UART/USB CDC──►   serial bytes
       │                                       │
       │ (firmware-side: see                   ▼
       │  firmware-stm32/                  WireDecoder
       │  or firmware/)                    (wire.rs)
       │                                       │
       │                                       ▼
       │                                  Event stream:
       │                                  Block / Run /
       │                                  Log / Overrun /
       │                                  Started / Stopped
       │                                       │
       │                                       ▼
       │                                  Board::permute
       │                                  (permute.rs)
       │                                       │
       │                                       │ (pa, pb) →
       │                                       │ (data, dc, cs)
       │                                       ▼
       │                                  BusDecoder
       │                                  (bus_decoder.rs)
       │                                  DC-edge framing
       │                                       │
       │                                       │ Frame { cmd, data }
       │                                       ▼
       │                          (data: u16, is_data: bool)
       │                                    │
       │                    ┌───────────────┴───────────────┐
       │                    │                               │
       │                    ▼                               ▼
       │              BusDecoder                      Glyph decoder
       │              (bus_decoder.rs)                (decoder.rs)
       │              DC-edge framing                 own 8080 framing,
       │                    │                         pixel runs,
       │                    │ Frame                   template match
       │                    ▼                               │
       │              Framebuffer                           │ row values
       │              (framebuffer.rs)                      │
       │              (egui display)                        │
       │                    │                               │
       │                    └───────────────┬───────────────┘
       │                                    ▼
       │                                  Shared{fb,log,values}
       │                                       │
       │                                       ▼
       │                                  eframe / egui UI
```

## Threads

Two threads, one shared `Mutex<Shared>` between them:

1. **eframe UI thread (main thread).** Renders the framebuffer texture
   and the activity log every ~33 ms. Pure read of `Shared`.
2. **Reader thread.** Owns the serial port (or the replay file). Reads
   raw bytes, drives the wire/permute/bus/glyph pipeline, mutates
   `Shared` under the lock.

The reader thread blocks in a 50 ms-timeout serial read loop. The
viewer hard-exits on Ctrl-C / window close rather than trying to
join the reader, because the reader is parked in I/O.

## Connect-time sync handshake

The wire protocol is binary and stateful; once the host loses byte
alignment, every subsequent byte looks like a fresh tag and the
parser fails. The handshake establishes a clean starting point every
time the viewer opens the port:

1. **DTR/RTS reset pulse** (`reader_loop` open path).
   - `DTR=true`  → BOOT0 low (run user code).
   - `RTS=true`  → NRST low (reset asserted).
   - 20 ms.
   - `RTS=false` → NRST released.
   - 250 ms boot delay.

   This forces the F103 into a known firmware state regardless of
   what the chip was doing before. The polarity is FTDI-specific
   (some adapters/EEPROMs invert); the values in the code are
   correct for the current bench rig.

   Harmless on USB-CDC boards (Pico) where DTR/RTS aren't wired to
   anything.

2. **Drain to STOPPED ack** (`sync` → `drain_until_quiet`).
   Send `0x02` (STOP). Read until the port goes quiet (one 50 ms
   read-timeout window with no bytes). Verify `0xFC` appeared
   somewhere in the drained bytes — that's the STOPPED ack the
   firmware always sends in reply, even when already stopped.

   If `0xFC` was seen, the firmware is now confirmed in STOPPED
   state and the byte stream is at a frame boundary.

3. **Send START** (`sync`).
   Send `0x01`. The firmware will reply with `0xFB` STARTED ack
   followed by sample frames. The main loop's WireDecoder handles
   both — `0xFB` surfaces as `Event::Started` and gets logged; the
   subsequent BLOCK/RUN frames are normal data.

   No explicit FB check — anything wrong here surfaces as a wire
   parse error in the main loop.

## Buffering / backpressure

The host doesn't try to buffer aggressively. Each stage is
synchronous and consumes input as fast as the previous stage
produces it:

- **OS tty buffer**: kernel's serial driver. ~4 KiB.
- **`buf: [u8; 4096]` in reader_loop**: per-read scratch. Not a
  persistent buffer; just the destination of one `read()`.
- **`WireDecoder::buf: Vec<u8>`**: holds partial frames across
  reads. Grows only when a frame doesn't fit in one read — typically
  empty after each `feed()` call. Critical: this is what makes the
  parser robust to read() boundaries falling mid-frame.
- **`BusDecoder::current: Option<Frame>`**: at most one
  in-progress 8080 transaction (the active command's accumulating
  payload). Flushed on every DC=0 sample.
- **`Framebuffer::pixels: Vec<u16>`**: 320×480 = 307 KB persistent
  framebuffer state.

If the firmware out-paces what the host can read+parse, the kernel
buffer fills first. Once it overflows, the kernel drops bytes
silently → torn frames → wire parse error → reader thread exits.
We rely on the firmware's atomic-frame sink to keep the *firmware
side* drop-free; we rely on USB CDC / UART pacing to keep the
*host side* drop-free.

## Modules

| File                          | Responsibility                                              |
|-------------------------------|-------------------------------------------------------------|
| [`main.rs`](../host/src/main.rs)             | Threading, sync handshake, UI scaffolding.   |
| [`wire.rs`](../host/src/wire.rs)             | Tagged-frame parser. Incremental, byte-stream input → `Event`s. |
| [`permute.rs`](../host/src/permute.rs)       | Per-board `(pa, pb) → (data, dc, cs)` reorder.       |
| [`bus_decoder.rs`](../host/src/bus_decoder.rs) | DC-edge 8080 bus framing → `Frame { cmd, data }`. |
| [`framebuffer.rs`](../host/src/framebuffer.rs) | Replays 0x2A/0x2B/0x2C transactions into a 320×480 RGB565 buffer. Surfaces window-sized writes as glyph candidates. |
| [`decoder.rs`](../host/src/decoder.rs)       | Matches glyph windows against baked-in templates, assembles per-row values (pm25, tvoc, co2, temp, humidity). See [`decoder_design.md`](decoder_design.md). |
| [`templates/`](../host/templates/)           | Source PNGs for the glyph templates; baked into the binary by `build.rs`. |

## CLI

```
viewer --port /dev/ttyUSB0 --board f103
viewer --port /dev/ttyACM0 --board pico   # default
viewer --replay capture.bin               # offline replay of a raw dump
viewer --dump-dir /tmp/glyphs/            # write every detected glyph as PNG
```

`--board` selects the permute function. Add a new board by adding a
variant + `permute_<name>` in `permute.rs`.

## Diagnostics

See [`host/README.md`](../host/README.md) for the usbmon recipe and
the STATS query via `printf '\x04' > /dev/ttyUSB0`.
