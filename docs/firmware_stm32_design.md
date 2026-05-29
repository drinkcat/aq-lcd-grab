# `firmware-stm32/` design

Capture firmware for the STM32F103C8 (Blue Pill bench rig today, fab'd
PCB later). Samples the 8080 display bus on every WR
strobe and ships the samples to the host over UART using the tagged
wire protocol in [`docs/wire_protocol.md`](wire_protocol.md).

## Pipeline

```
                                            ┌──────────────────────┐
  WR rising edge                            │ async runtime: 4 fut │
  ───────────────────►                      │  joined together     │
            │                               │                      │
            ▼                               │  led_fut             │
        TIM2 ETR ─── input cap on CH1+CH2 ──┤  (just toggle LED)   │
            │ ECE=1                         │                      │
            │                               │  cap_fut ◄───────┐   │
            ├──CC1IF→CC1DE→DMA1_CH5─►PA_BUF │   │              │   │
            │                               │   │  capture.drain│  │
            ├──CC2IF→CC2DE→DMA1_CH7─►PB_BUF │   │              │   │
            │                               │   ▼              │   │
            ▼                               │  Encoder.feed()  │   │
        peripheral                          │   │              │   │
                                            │   ▼              │   │
                                            │  QueueSink       │   │
                                            │  (atomic frames) │   │
                                            │   │              │   │
                                            │   ▼              │   │
                                            │  TX_QUEUE        │   │
                                            │  (Channel<u8,    │   │
                                            │   4096>)         │   │
                                            │   │              │   │
                                            │   ▼              │   │
                                            │  tx_fut ─USART1─►host│
                                            │                  ▲   │
                                            │  rx_fut ─USART1──┤   │
                                            │   │              │   │
                                            │   ▼              │   │
                                            │  CMD_QUEUE ──────┘   │
                                            │  (HostCmd)           │
                                            └──────────────────────┘
```

## Capture front-end (`capture.rs`)

The WR strobe drives **PA0 = TIM2_ETR**. TIM2 runs in
external-clock mode 2 (`SMCR.ECE = 1`), so the timer's counter
increments on every rising edge of WR — no CPU involvement.

Two timer events fire on the **same** WR edge, by configuring CC1
and CC2 in input-capture mode both pointing at TI1 (TIM2's
`CH1_ETR` is one physical pin that all of ETR, CH1, and CH2 can
read):

- **CC1 input-capture** → `CC1IF` → `CC1DE` → DMA1_CH5 transfers
  `GPIOA->IDR` (low 16 bits) into `PA_BUF` (ring buffer, 1024 ×
  u16).
- **CC2 input-capture** → `CC2IF` → `CC2DE` → DMA1_CH7 transfers
  `GPIOB->IDR` (low 16 bits) into `PB_BUF` (ring buffer, 1024 ×
  u16).

Both rings are embassy `ReadableRingBuffer`s in circular mode. The
CPU only touches them through `Capture::drain`, which reads
`min(available_pa, available_pb)` paired samples — so PA[i] and PB[i]
always correspond to the same WR edge.

Why dual input-capture (rather than UEV + CC1, which is what the
original TIM1 design used):

- **TIM1** has separate ETR and CH1 pins (PA12 and PA8), so CC1 in
  output-compare mode + UEV-driven DMA worked there.
- **TIM2** multiplexes ETR and CH1 onto one pin (`PA0 = CH1_ETR`).
  Output-compare CC1 generates a "compare-match" event on a CNT
  transition; with `ARR = 0` (UEV every edge), CNT never actually
  transitions — it overflows-to-zero instantly — and `CC1IF` never
  fires. Input-capture sidesteps this by triggering directly off
  the input pin's edge.
- We discovered empirically that `ARR = 0 + ECE + UDE`'s DMA never
  fires at all on F103 (a quirk we didn't fully understand). Dual
  input-capture doesn't depend on UEV, so it's robust either way.

## Encoder (`wire.rs`)

Stateful run-length encoder. Feed one packed `u32` sample (low 16 =
`pa`, high 16 = `pb`); the encoder buffers consecutive identical
samples as a run and emits one of:

- `tag=0x01 BLOCK`: a batch of up to 255 distinct samples.
- `tag=0x02 RUN`: 2..255 repeats of one sample.

Plus helpers for log frames (`0xFE`), overrun (`0xFD`), START/STOP
acks (`0xFB`/`0xFC`).

The encoder calls `sink.commit_frame()` at every frame boundary so
the sink can publish the frame atomically.

## QueueSink (atomic-frame guarantee)

`QueueSink` is *the* reason the host can reliably parse a stream
under load.

It stages each frame into a 1024-byte scratch buffer. Per byte,
`push()` appends to scratch. On `commit_frame()`:

- If scratch overflowed (frame was larger than 1024 B — shouldn't
  happen, but guard), discard the whole frame.
- If `TX_QUEUE` has at least `frame_size` free slots, push the
  whole frame in one go (`try_send` per byte, but pre-checked
  capacity so each `try_send` is guaranteed to succeed).
- Otherwise, discard the whole frame and increment a dropped-bytes
  counter (surfaced via STATS).

The host parser depends on never seeing a torn frame. Without this
atomic guarantee, when the queue fills mid-frame, half the frame
bytes would land on the wire and the rest would be lost — the host
would parse garbage and desync. (We learned this the hard way on
the Pico — see the commit history.)

## Async tasks (`main.rs`)

Four futures joined with `join(join3(led, tx, rx), cap)`:

| Task     | What it does                                                            |
|----------|-------------------------------------------------------------------------|
| `led_fut`| Toggle PC13 at 1 Hz (STOPPED) / 5 Hz (STREAMING). Reads `STREAMING` atomic. |
| `tx_fut` | Block on first byte from `TX_QUEUE`, opportunistically batch up to 256 B, ship via `BufferedUart` write. |
| `rx_fut` | Block on UART read of 1 byte. On `0x01`/`0x02`, push `HostCmd::Start`/`Stop` into `CMD_QUEUE`. |
| `cap_fut`| Main loop: drain `CMD_QUEUE` (emit STARTED/STOPPED acks unconditionally), drain capture rings (feed encoder when STREAMING), yield once per iteration, sleep 2 ms when idle. |

## Buffers / sizes / pacing

| Buffer / queue       | Size                          | Purpose                                                     |
|----------------------|-------------------------------|-------------------------------------------------------------|
| `PA_BUF` / `PB_BUF`  | 1024 × u16 each (4 KiB total) | DMA capture rings. ~1.5 ms of headroom at 667 kHz peak WR.  |
| `QueueSink.buf`      | 1024 B                        | One-frame staging scratch.                                  |
| `TX_QUEUE`           | 4096 B                        | Encoder→TX-task byte queue. ~44 ms at 92 kB/s drain rate.   |
| `UART_TX_BUF`        | 4096 B                        | embassy-stm32 `BufferedUart` interrupt-side TX buffer.      |
| `UART_RX_BUF`        | 64 B                          | RX buffer; commands are 1-byte and rare.                    |
| `CMD_QUEUE`          | 8 × HostCmd                   | RX→cap-task command FIFO.                                   |

**Drain cadence.** Capture task pulls up to `DRAIN_CHUNK = 1024`
paired samples per iteration, feeds them all into the encoder, then
flushes any partial run/block so the wire latency is bounded by the
2 ms idle sleep rather than waiting for a 255-sample block to fill.

**Yielding under noise.** Without explicit `yield_now()` per
iteration, a noisy floating ETR (bench rig with no target connected)
would feed enough phantom edges that `capture.drain` always returns
non-zero → the inner loop never sleeps → LED/RX/TX tasks starve.
The yield guarantees one scheduler tick per outer iteration.

## State machine

Two states: `Stopped`, `Streaming`. Boot is `Stopped`. Transitions:

- `HostCmd::Start`: encoder.reset(); state = Streaming; STREAMING = true;
  encode STARTED ack. Always emits the ack, even when already Streaming
  (host's sync handshake assumes STOP/START always produces an ack).
- `HostCmd::Stop`: encoder.flush(); state = Stopped; STREAMING = false;
  encode STOPPED ack.

In `Stopped`, the cap task still drains the capture rings (so they
don't overflow), but doesn't feed the encoder.

## Connect-side handshake (host view)

The host's reset pulse on connect (DTR/RTS — see
[`host_design.md`](host_design.md)) gives the F103 a known starting
state. The firmware then:

1. Boots.
2. Emits a UTF-8 log frame: `"aq-lcd-grab stm32 firmware booted,
   awaiting START"`.
3. Sits in `Stopped` until a `0x01` byte arrives on USART1.
4. Responds with `0xFB` and starts streaming sample frames.

## Diagnostics surfaced via the wire

- **tag=0xFD OVERRUN**: PIO ring overran, lost N WR edges. (Embedded
  in the cap_fut overrun-reporting block.)
- **tag=0xFE LOG**: UTF-8 status string. Used for boot banner and
  the periodic `"idle"` heartbeat (every ~5 s while streaming).

`QueueSink::dropped` tracks TX-pipe drops but isn't surfaced
automatically yet (would need a STATS command path like the Pico has).
