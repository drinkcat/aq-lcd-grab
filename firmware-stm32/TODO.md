# firmware-stm32 TODO

## Zero-copy DMA → encoder hand-off (upstream embassy-stm32)

The capture path currently stages each batch of samples through two
stack-resident `[u16; DRAIN_CHUNK]` scratch buffers before packing
them into u32 samples and feeding the encoder:

```
DMA ring → pa_buf/pb_buf (stack copy via embassy's `read`)
         → for i in 0..n { encoder.feed(pa | pb<<16) }
```

We can't get rid of the copy with embassy-stm32 0.5's public API.
`ReadableRingBuffer` only exposes `read(&mut [W])` (copy out) and
`read_exact(&mut [W])` (async copy out). The internal
`ReadableDmaRingBuffer` has `read_buf(offset) -> W` (volatile read
of one element at the current DMA-relative offset) and a private
`read_index.advance(n)` — but neither is public.

**Proposed upstream API** (would let us iterate volatile-reads of
the DMA buffer straight into the encoder, no staging):

```rust
impl<'a, W: Word> ReadableDmaRingBuffer<'a, W> {
    /// Return up to `max` readable elements as an iterator that
    /// volatile-reads from the DMA buffer. Does NOT advance the
    /// read index — call `consume(n)` after to commit how many
    /// were actually used.
    pub fn peek<'b>(
        &'b mut self,
        dma: &'b mut impl DmaCtrl,
        max: usize,
    ) -> Result<impl Iterator<Item = W> + 'b, Error>;

    pub fn consume(&mut self, n: usize);
}
```

Caller usage in our cap loop:

```rust
let pa_iter = pa_ring.peek(&mut pa_dma, DRAIN_CHUNK)?;
let pb_iter = pb_ring.peek(&mut pb_dma, DRAIN_CHUNK)?;
let mut n = 0;
for (pa, pb) in pa_iter.zip(pb_iter) {
    let sample = (pa as u32 & PA_MASK as u32) | ((pb as u32 & PB_MASK as u32) << 16);
    encoder.feed(sample, &mut sink);
    n += 1;
}
pa_ring.consume(n);
pb_ring.consume(n);
```

Saves 2 × DRAIN_CHUNK × 2 B = 512 B of stack on the cap_fut and
removes the per-batch memcpy. The volatile reads happen anyway —
they just go direct to a register instead of to a stack slot.

Until upstream adopts this, live with the staging copy — it's
~256 cycles per batch, negligible vs. the encoder + UART per-byte
costs.

Tracking links to fill in if/when filed:
- embassy issue: TBD
- embassy PR: TBD
