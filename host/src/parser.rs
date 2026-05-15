//! Parse the firmware's text dump format into a stream of samples.
//!
//! The firmware logs one capture as:
//!
//!     waiting for 4096 samples on WR (GPIO 18)…
//!     captured 4096 samples, dumping all:
//!     [0000] 00000 00000 00000 00000 00000 28000 00000 08000
//!     [0008] 00000 24000 2c000 00000 28000 04000 18000 30000
//!     ...
//!     [4088] ... ...
//!     dump done
//!
//! We tolerate the framing chatter and just pick up 5-hex-digit tokens
//! between `[NNNN]` index markers.

use crate::sample::Sample;

/// Parse a single line from the firmware. Returns the samples found.
/// Returns `None` if the line doesn't look like a `[NNNN] ...` payload.
pub fn parse_line(line: &str) -> Option<Vec<Sample>> {
    let trimmed = line.trim();
    // Strip the `[NNNN] ` prefix.
    let rest = trimmed.strip_prefix('[')?;
    let (_idx, payload) = rest.split_once(']')?;

    let mut samples = Vec::new();
    for tok in payload.split_whitespace() {
        // Each token should be 5 hex digits.
        if tok.len() != 5 || !tok.chars().all(|c| c.is_ascii_hexdigit()) {
            return None;
        }
        let raw = u32::from_str_radix(tok, 16).ok()?;
        samples.push(Sample::from_raw(raw));
    }
    Some(samples)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_line() {
        let line = "[0008] 00000 24000 2c000 00000 28000 04000 18000 30000";
        let samples = parse_line(line).unwrap();
        assert_eq!(samples.len(), 8);
        // [0]: raw 0x00000 -> data 0, dc 0, cs 0
        assert_eq!(samples[0].data, 0x0000);
        // [2]: raw 0x2c000 -> data 0xc000, dc 0, cs 1
        assert_eq!(samples[2].data, 0xc000);
        assert!(!samples[2].dc);
        assert!(samples[2].cs);
    }

    #[test]
    fn ignores_chatter() {
        assert!(parse_line("dump done").is_none());
        assert!(parse_line("captured 4096 samples, dumping all:").is_none());
    }
}
