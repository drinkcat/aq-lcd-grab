# Capture-board wire protocol (STM32 ↔ host)

The STM32F103 captures the target device display bus and ships frames out
USART1 (PA9 TX, PA10 RX) at **921600 8N1**. This document describes
the on-wire format in both directions.

## Design constraints

- **Source rate.** WR strobe runs at ~667 kHz peak. Per WR edge we
  capture two 16-bit port reads (`GPIOA->IDR` + `GPIOB->IDR`) → 4 bytes
  raw per sample, ~2.67 MB/s sustained.
- **Sink rate.** USART1 at 921600 baud ≈ 92 kB/s. ~30× short of raw.
  Mitigated by run-length encoding: long uniform pixel fills compress
  ~3000:1; brief command sequences carry their own overhead.
- **Firmware is dumb on purpose.** It does **not** permute physical
  pins to logical DB bits, **does not** extract DC/CS, **does not**
  parse command bytes. It captures raw `(pa, pb)` pairs and merges
  consecutive identical pairs into runs. Everything else is the
  host's job — including DC extraction (just one bit of `pb` after
  permutation).
- **Host applies the permutation table** (`LOGICAL_TO_PHYSICAL[16]`)
  generated from the SKiDL netlist (pcb_spec.md §Q17).
- **Explicit start/stop.** Firmware boots quiet (no data frames). Host
  sends `START` to begin streaming and `STOP` to halt. Firmware
  acknowledges both. Resync is implicit: `STOP` → drain tty → `START`
  yields a known-clean stream. No magic bytes are needed.

## Direction: STM32 → host (data path)

Every frame is `[tag u8] [body...]`. The tag implies (or the body
encodes) the body length. There are no magic / sync bytes.

### tag = 0x01 — block of unique events (variable body)

N consecutive WR edges where no two adjacent edges had identical
`(pa, pb)`. Bundled into one frame to amortise the per-frame header
across many bytes when the bus is full of unique samples (e.g. mixed
pixel data with no repeats).

```
[0x01]  sample[0]  sample[1]  …  sample[n-1]  0xffff_ffff
        └─────── u32 LE each ───────┘         └ sentinel ┘
```

- The sample list is terminated by the sentinel value
  `0xffff_ffff`. There is no leading count, so the encoder can
  stream samples straight to the wire without buffering to know
  `n` up front.
- `0xffff_ffff` is never a legal sample: captures are masked to
  18 bits (`raw & 0x0003_ffff`) on both boards, so the high 14
  bits are always 0. The sentinel is therefore unambiguous.
- Each sample is `pa | pb << 16` — for the STM32 that is
  `GPIOA->IDR | GPIOB->IDR << 16` at one WR edge.
- Body = 4·(n + 1) bytes.

A run of identical samples in the middle of a block forces the
block to flush (tag=0x01 ends, tag=0x02 starts). Within a block,
all consecutive pairs differ.

### tag = 0x02 — run-length event (6-byte body)

N consecutive WR edges produced identical `(pa, pb)`. N ≥ 2.

```
[0x02] [n_lo] [n_hi] [pa_lo] [pa_hi] [pb_lo] [pb_hi]
       └── u16 LE ──┘ └── u16 LE ──┘ └── u16 LE ──┘
```

- `n` = number of WR edges represented (≥ 2, ≤ 65535).

A run longer than 65535 edges is split into multiple tag=0x02
frames back-to-back.

### tag = 0x03 — alternating runs (variable body)

A sequence of same-sample runs that strictly alternate between
two distinct sample values. Optimised for B&W pixel rendering
on the target device where the bus toggles between `0xFFFF` and
`0x0000` every few-to-tens of WR edges — naïve RLE wastes
~7 bytes of header per toggle; this frame collapses each toggle
to 1 byte at the cost of one shared 9-byte header per sequence.

```
[0x03] [a_lo a_hi pa_lo pa_hi]  [b_lo b_hi pb_lo pb_hi]  [run_lens...] [0x00]
       └── val_a u32 LE ─────┘  └── val_b u32 LE ─────┘  └── u8 each ─┘
```

- `val_a`, `val_b` = the two distinct sample values that
  alternate. Run 0 is `val_a`, run 1 is `val_b`, run 2 is
  `val_a`, etc.
- `run_lens[i]` = u8 length of run *i*. Runs are always ≥ 2
  (lone samples never participate in REPEAT2) and ≤ 255 in
  this encoding; runs longer than 255 force a REPEAT2 flush
  and re-enter normal RUN encoding.
- `0x00` terminates the run-length list (0 is never a valid
  run length, so it's unambiguous). The frame contains ≥ 3
  runs in practice — below that the encoder emits individual
  RUN frames instead.
- Body size: 4 + 4 + n + 1 bytes for `n` runs.

A third distinct sample value, or a run > 255, terminates
the REPEAT2 frame; the encoder flushes what it has and
resumes with normal RUN/BLOCK encoding.

### tag = 0xFD — overrun marker (4-byte body)

Inserted when the capture path detected that DMA ring overruns lost
samples. Helps the host mark gaps.

```
[0xFD] [dropped_lo] [dropped_mid] [dropped_hi] [dropped_top]
       └─────── u32 LE ─────────────────────────┘
```

- `dropped` = count of WR edges (samples) the firmware knows it lost
  since the last overrun frame.

### tag = 0xFE — log line (variable body)

Out-of-band UTF-8 text from the firmware (boot banner, periodic stats).

```
[0xFE] [len_lo] [len_hi] [utf8 bytes × len]
       └── u16 LE ──┘
```

`len` ≤ 256; longer messages are truncated by the firmware. Trailing
newline is **not** included.

### tag = 0xFB — STARTED acknowledgement (0-byte body)

Sent by firmware in response to a host `START` command. Marks the
beginning of a streaming session — every byte after this is a fresh
frame.

```
[0xFB]
```

### tag = 0xFC — STOPPED acknowledgement (0-byte body)

Sent by firmware in response to a host `STOP` command. Marks the end
of a streaming session. After this byte, the firmware is silent
until the next `START` (apart from possible boot-time log lines if
the host issues `STOP` very early — see below).

```
[0xFC]
```

### tag = 0xFA — drain tick (14-byte body)

Out-of-band heartbeat emitted ~100×/sec while STREAMING. Gives
the host wall-clock + backlog + throughput telemetry without
per-sample timestamping — enough to plot arrival rate, drain
throughput, ring fill level, and compression ratio over time.
Lives in the `0xFX` out-of-band range alongside OVERRUN / LOG /
the acks, not the low data-tag range.

```
[0xFA] [t_us:u32 LE] [dt_us:u16 LE] [n_drained:u16 LE] [n_pending:u16 LE] [bytes_out:u32 LE]
```

- `t_us` = firmware Instant (low 32 bits, µs). Wraps every ~71 min.
- `dt_us` = wall-clock duration of the TICK window (`t1 - t0`).
- `n_drained` = samples consumed across the window.
- `n_pending` = `available()` at emit time — the backlog still
  sitting in the PIO/DMA ring. Approaches the ring size when the
  firmware can't keep up.
- `bytes_out` = bytes the firmware enqueued onto the stream
  during this TICK window, **excluding** TICK frames themselves.
  **Per-window delta, not cumulative.** Compare against
  `n_drained` × 4 for the effective compression ratio over the
  window.

TICKs are emitted every ~10 ms but *only* when the window had
something to report — if `n_drained`, `n_pending`, and
`bytes_out` are all 0 the TICK is suppressed, keeping the wire
silent during idle stretches. The next non-zero TICK then covers
the full silent gap via `dt_us`.

### Reserved tags

`0x00`, `0x04`–`0xF9`, `0xFF` are reserved. Unknown tags are a fatal
parse error — the host should `STOP`, drain, and `START` over.

## Direction: host → STM32 (control path)

Single-byte commands. No body, no echo on the same byte — firmware
replies via tagged frames on the data path.

| Byte | Command  | Firmware reply                                 | Purpose                  |
|------|----------|------------------------------------------------|--------------------------|
| 0x01 | START    | finishes nothing in flight (was quiet), `[0xFB]` | begin streaming sessions |
| 0x02 | STOP     | finishes any in-flight frame, `[0xFC]`         | halt streaming, drain    |
| 0x03 | LOG_TEST | `[0xFE] "ping"` log frame                      | round-trip health check  |
| 0x04 | STATS    | one or more `[0xFE]` lines                     | dump capture counters    |

`START` while already started, and `STOP` while already stopped, are
both no-ops apart from the corresponding ack frame. This lets the
host force-reset to a known state by sending `STOP` then `START`
regardless of what state it thought firmware was in.

Future expansion (change baud, query firmware version) will use
bytes from 0x05 upward.

## Sync protocol

On connect:

1. Host drains and discards anything sitting in the OS tty buffer.
2. Host sends `0x02` (STOP). This is a no-op in normal operation
   (firmware boots stopped) but covers the case where a stale
   firmware was already streaming.
3. Host reads bytes, discarding everything until it sees a `0xFC`
   byte.
4. Host sends `0x01` (START).
5. Host reads bytes, discarding everything until it sees `0xFB`.
6. Any byte after `0xFB` starts a fresh frame.

During steady-state operation, if the host's decoder hits a
malformed frame (unknown tag, impossible `len`, truncated body), it
repeats steps 2–6 to recover.

The firmware's state machine:

- **STOPPED** (initial): TIM1 + DMA are running and accumulating
  into the ring buffer, but the streaming task does not emit data
  frames. Log frames (`tag=0xFE`) may still be sent. RLE accumulator
  is cleared on entry.
- **START** transitions to **STREAMING**, sends `[0xFB]`, begins
  emitting data frames.
- **STOP** finishes any in-flight frame body, sends `[0xFC]`, drops
  RLE state, transitions back to **STOPPED**.

## Host-side decode

The authoritative decoder is `host/src/wire.rs` (`Decoder` /
`parse_one`). After decoding, the host applies the permutation
table to turn `(pa, pb)` into `(data: u16, dc: bool, cs: bool)`,
then feeds the result into the existing protocol decoder (which
expects ILI9488-style command bytes + RGB565 pixel data).

## Bandwidth budget

At 921600 baud (≈ 92 kB/s after 10-bit framing per character):

| Scenario              | Frames/sec      | Bytes/sec | Headroom |
|-----------------------|-----------------|-----------|----------|
| Idle (no WR edges)    | ~0              | ~0        | 100%     |
| Long pixel fill (RLE) | ⌈samples/255⌉   | ~6 B/255 px | ≫100% |
| Mixed pixels (run=4)  | 167k single+run | ~830 kB/s | -800%    |
| All unique pixels     | 667k single     | ~3.3 MB/s | -3500%   |

The last two rows are pathological. The capture board's purpose is
to record the target's actual display traffic, which is mostly large
solid fills (sensor-value backgrounds) and short command bursts —
the median ratio in practice should be well under 92 kB/s. The
`tag=0xFD overrun` frame is the firmware's safety valve when reality
disagrees.
