// FNV-1a 64-bit hash — shared between build.rs (template baking) and
// decoder.rs (runtime matching). build.rs includes this file directly
// via include!(); decoder.rs uses it as a normal module.
//
// The hash must be identical in both contexts, so any change here
// regenerates the template table automatically (build.rs watches src/).

pub const FNV_OFFSET: u64 = 0xcbf29ce484222325;
pub const FNV_PRIME: u64 = 0x100000001b3;

pub fn fnv_init() -> u64 {
    FNV_OFFSET
}

pub fn fnv_mix(hash: u64, byte: u8) -> u64 {
    (hash ^ byte as u64).wrapping_mul(FNV_PRIME)
}
