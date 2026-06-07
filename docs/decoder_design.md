# Decoder design

The decoder (`host/src/decoder.rs`) matches captured glyph windows
against a baked-in template table and assembles per-row sensor values
(pm25, tvoc, co2, temp, humidity) once each row settles.

## Goal: stream-oriented, embedded-ready

The decoder consumes the raw `(data: u16, is_data: bool)` byte stream
— the same level that the wire encoder operates at on the firmware
side — without depending on `BusDecoder` or `Framebuffer`. This means:

- No `Frame` assembly step needed before decoding.
- No dependency on `Framebuffer` for value extraction.
- No `std::time::Instant` — timing is driven by an external tick.
- Fixed-size internal state (no per-glyph heap allocation for the
  pixel buffer) — a prerequisite for running on the STM32F1.

The host's `BusDecoder` and `Framebuffer` are kept for the egui
display only; the decoder independently does its own 8080 framing at
the byte level.

## Pipeline position

```
  permute.rs
  (pa, pb) → (data: u16, is_data: bool)
      │
      ├──────────────────────────────────────────►  BusDecoder + Framebuffer
      │                                             (egui display only)
      │
      ▼
  Decoder::feed(data, is_data)
      │  own 8080 framing: is_data==false → new cmd,
      │                    is_data==true  → data byte / pixel word
      │  tracks 0x2A / 0x2B / 0x2C commands
      │  accumulates pixels as PixelRuns
      │  matches complete windows against templates
      ▼
  Option<DecodeOut>  { glyph: Option<GlyphMatch>, completed_rows: Vec<RowReport> }
```

On idle (serial read timeout), the caller pumps `Decoder::tick()` to
surface rows that have stopped updating.

## Internal 8080 framing

The decoder replicates the minimal framing logic from `BusDecoder`:

- `is_data == false`: a new command byte arrives. Finalise any
  in-progress command (may trigger a match if it was a complete
  MEMORY_WRITE window), then record the new command byte.
- `is_data == true`: a data word for the current command. The
  interpretation depends on which command is active:
  - `0x2A` (SET_COLUMN_ADDRESS): accumulate 4 bytes → `col_start`, `col_end`.
  - `0x2B` (SET_ROW_ADDRESS): accumulate 4 bytes → `row_start`, `row_end`.
  - `0x2C` (MEMORY_WRITE): if window is relevant, accumulate pixel runs.
  - `0x3C` (MEMORY_WRITE_CONTINUE): extend the current pixel run.
  - anything else: discard.

The decoder keeps only this framing state — no `Vec<u16>` payload
buffer — so it allocates nothing for irrelevant commands.

## Pixel hashing

As each pixel arrives the decoder binarizes it (foreground = pixel ≠
background, where background is the first pixel of the window) and
extends the current bg or fg run. When a run ends (the fg/bg state
flips), its length is mixed into a running hash. No pixel buffer is
needed — only the hash state, the background color, and the current
run length are kept across pixels.

```
per pixel:
    is_fg = (pixel != bg)
    if is_fg == current_run_is_fg:
        run_len += 1
    else:
        hash = hash_mix(hash, run_len)
        run_len = 1
        current_run_is_fg = is_fg
// at window end: mix in the final run
hash = hash_mix(hash, run_len)
```

This is:
- **O(W·H) time, O(1) space**: one comparison per pixel, one hash mix
  per run boundary (far fewer than one per pixel for typical glyphs).
- **Efficient on embedded**: a ~40×61 digit glyph has ~20–30 runs,
  so only ~20–30 hash steps per glyph instead of ~2440.
- **Embedded-friendly**: the only state is `bg: u16`, `run_len: u16`,
  `run_is_fg: bool`, and one hash accumulator word.

## Window tracking

The decoder maintains its own `col_start/col_end/row_start/row_end`
state, updated as `0x2A` / `0x2B` data bytes arrive. When a `0x2C`
command starts:

1. Compute `w = col_end - col_start + 1`, `h = row_end - row_start + 1`.
2. Check whether the window overlaps any known metric region (`RowDef`).
   Overlap = `col_start..=col_end` intersects `r.x_min..r.x_max`
   AND `row_start..=row_end` intersects `r.y_min..r.y_max`.
3. If no overlap: discard all pixels silently (no allocation).
4. If overlap: open a `PendingWindow` and accumulate pixel runs until
   `pixel_count == w * h`, then attempt a template match.

Only exact-fill writes (pixel count == window area) are candidates —
same rule as before.

### Display coordinate flip

The capture target mounts the panel upside-down. Display-space
coordinates (used for `RowDef` hit-testing and `GlyphMatch` output)
are derived as:

```
disp_x = WIDTH  - col_start - w   (= WIDTH  - col_end - 1)
disp_y = HEIGHT - row_start - h   (= HEIGHT - row_end - 1)
```

The same flip applies when reconstructing the packed-bit mask for
template matching: pixels are iterated in reverse order so the mask
is in display orientation, matching how the templates were captured.

## Template matching

`build.rs` is extended to compute a hash for each template (same
algorithm as the runtime) and emit it alongside the label:

```rust
struct Template { w: u16, h: u16, label: &'static str, hash: u64 }
```

The packed-bit `mask` field is dropped from the runtime table (it can
be kept in the build script for debugging but isn't needed at runtime).

At runtime, when a window completes:

1. Compare `(w, h, hash)` against the template table — O(N) scan, but
   N is small (~20 templates) and each comparison is three integer
   ops.
2. On a match, return the label. On no match, return `"#"`.

Hash collisions between distinct glyphs of the same size are possible
but astronomically unlikely with a 64-bit hash over ~2500 pixels.
`build.rs` asserts at compile time that no two templates of the same
size share a hash, making collisions a build error rather than a
silent misread.

### Hash algorithm

FNV-1a 64-bit is a good fit: it's a single multiply-xor per input
byte, no division, no lookup table — suitable for `no_std` and fast on
Cortex-M. The input to the hash is the reversed binarized pixel stream
(one bit per pixel, same display-orientation convention as the old
packed-bit masks), fed as individual bytes with the remaining bits
zero-padded.

Alternatively, a simple polynomial hash over the fg-bit stream (one
u32 multiply + xor per pixel) is even cheaper and equally effective at
this data size.

## Flush on idle

The decoder has no internal timer. Instead it exposes a `flush()`
method that the caller invokes when it detects idleness — on the host
that's a serial read timeout, on the STM32 it's the drain loop's
10 ms wait expiring with no new samples. `flush()` iterates dirty rows
and emits a `RowReport` (or calls a sink callback) for each one, then
clears the dirty flag.

This keeps all timing policy in the main loop, where it belongs, and
keeps the decoder itself completely stateless with respect to time.

## Fixed-size digit storage

Each row holds at most `MAX_DIGITS_PER_ROW = 8` digit slots, keyed by
display-space x coordinate and stored in sorted order. This bounds the
per-row allocation to a fixed array `[Option<(u16, &'static str)>; 8]`,
with `n_digits: usize` tracking the live count. Insertion maintains
sort order with a linear scan (array is tiny).

## Public API

```rust
impl Decoder {
    pub fn new() -> Self;
    /// Feed one permuted bus sample. Returns a DecodeOut when a glyph
    /// window completes; None otherwise.
    pub fn feed(&mut self, data: u16, is_data: bool) -> Option<DecodeOut>;
    /// Called by the main loop when idle. Emits RowReports for any dirty
    /// rows and clears their dirty flags.
    pub fn flush(&mut self) -> Vec<RowReport>;
}
```

`DecodeOut`, `GlyphMatch`, and `RowReport` are unchanged from before.

## Impact on host pipeline

`dispatch_event` in `main.rs` already calls `board.permute(sample)`
to produce `(data, is_data)` before feeding `BusDecoder`. The glyph
decoder taps in at the same point — each permuted sample goes to both
`bus.feed(data, is_data)` (for the display) and
`glyphs.feed(data, is_data)` (for value extraction). No structural
change to the event dispatch loop is needed beyond adding the second
call.

## Future: moving to no_std / STM32

The decoder is designed so it could move to the `wire` crate or a new
`no_std` decoder crate with minimal changes:

- The RLE hash step has no heap allocation — only a few scalar fields
  in `PendingWindow` (`bg`, `run_len`, `run_is_fg`, hash accumulator,
  pixel count).
- Replace `Vec<RowReport>` returns with a callback or a fixed-size
  output array.
- Template hashes are `&'static` data — no runtime allocation.

### Permutation cost on STM32

On the STM32 the DMA delivers raw `u32` samples directly. The full
GPIO permutation (unpacking scattered PA/PB bits into a clean 16-bit
data word + framing signals) runs on every sample and is non-trivial
at 667 kHz.

The decoder's `feed` interface should therefore accept `u32` directly
and apply only as much permutation as needed for each state:

- **Framing state** (waiting for `0x2A`/`0x2B`/`0x2C`): need `is_data`
  and the command byte — full permute required, but these are rare.
- **Mid-MEMORY_WRITE pixel state**: only need to know whether
  `data == bg`. If the permutation is linear (a fixed bit-shuffle with
  no arithmetic), it may be possible to pre-permute `bg` into raw GPIO
  space and compare directly against the raw `u32`, skipping the
  permute entirely for the hot pixel path.

This optimisation is left for the STM32 port; the host build uses the
existing `Board::permute` and passes `(data, is_data)` as today.
