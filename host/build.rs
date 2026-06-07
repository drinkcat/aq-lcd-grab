//! Bake `templates/<W>x<H>/<label>.png` into a hash table the decoder
//! can match against at runtime.
//!
//! Each template PNG is binarized (any non-black pixel = foreground).
//! The hash is FNV-1a 64-bit over the RLE run-length sequence of the
//! binarized pixel stream, iterated in reverse (display orientation —
//! same convention as the runtime accumulator in decoder.rs).
//!
//! Output emitted to `$OUT_DIR/templates_gen.rs`:
//!
//! ```ignore
//! pub struct Template { pub w: u16, pub h: u16, pub label: &'static str, pub hash: u64 }
//! pub static TEMPLATES: &[Template] = &[ ... ];
//! ```
//!
//! A compile-time assertion (via a generated `const` block) checks that
//! no two templates of the same size share a hash.

use std::path::{Path, PathBuf};

fn main() {
    let dir = PathBuf::from("templates");
    println!("cargo:rerun-if-changed={}", dir.display());
    println!("cargo:rerun-if-changed=src/fnv.rs");

    let mut entries: Vec<(u16, u16, String, u64)> = Vec::new();
    let size_dirs = std::fs::read_dir(&dir).expect("templates/ missing");
    for entry in size_dirs {
        let entry = entry.expect("read templates entry");
        let size_path = entry.path();
        if !size_path.is_dir() {
            continue;
        }
        let size_name = size_path
            .file_name()
            .and_then(|s| s.to_str())
            .expect("utf8 size name");
        let (w, h) = parse_size(size_name)
            .unwrap_or_else(|| panic!("bad size dir name {size_name:?} (expected WxH)"));
        println!("cargo:rerun-if-changed={}", size_path.display());

        for file in std::fs::read_dir(&size_path).expect("read size dir") {
            let file = file.expect("read entry");
            let path = file.path();
            if path.extension().and_then(|s| s.to_str()) != Some("png") {
                continue;
            }
            let stem = path
                .file_stem()
                .and_then(|s| s.to_str())
                .expect("utf8 stem")
                .to_string();
            println!("cargo:rerun-if-changed={}", path.display());
            let hash = hash_png(&path, w, h);
            entries.push((w, h, stem, hash));
        }
    }

    entries.sort_by(|a, b| (a.0, a.1, &a.2).cmp(&(b.0, b.1, &b.2)));

    // Verify no two templates of the same size share a hash.
    for i in 0..entries.len() {
        for j in (i + 1)..entries.len() {
            let (wi, hi, li, hi_hash) = &entries[i];
            let (wj, hj, lj, hj_hash) = &entries[j];
            if wi == wj && hi == hj && hi_hash == hj_hash {
                panic!(
                    "hash collision: templates {li:?} and {lj:?} at {wi}x{hi} share hash {hi_hash:#018x}"
                );
            }
        }
    }

    let out_dir = std::env::var_os("OUT_DIR").expect("OUT_DIR");
    let out_path = Path::new(&out_dir).join("templates_gen.rs");
    let mut src = String::new();
    src.push_str("pub struct Template { pub w: u16, pub h: u16, pub label: &'static str, pub hash: u64 }\n");
    src.push_str("pub static TEMPLATES: &[Template] = &[\n");
    for (w, h, label, hash) in &entries {
        src.push_str(&format!(
            "    Template {{ w: {w}, h: {h}, label: {label:?}, hash: {hash:#018x} }},\n"
        ));
    }
    src.push_str("];\n");
    std::fs::write(&out_path, src).expect("write templates_gen.rs");
}

fn parse_size(s: &str) -> Option<(u16, u16)> {
    let (w, h) = s.split_once('x')?;
    Some((w.parse().ok()?, h.parse().ok()?))
}

include!("src/fnv.rs");

fn hash_png(path: &Path, expected_w: u16, expected_h: u16) -> u64 {
    let img = image::open(path).unwrap_or_else(|e| panic!("open {}: {e}", path.display()));
    let img = img.to_luma8();
    let (w, h) = (img.width() as u16, img.height() as u16);
    assert_eq!(
        (w, h),
        (expected_w, expected_h),
        "{}: dim {w}x{h} != dir {expected_w}x{expected_h}",
        path.display()
    );

    // Binarize: non-black = foreground.
    let pixels: Vec<bool> = img.pixels().map(|px| px.0[0] != 0).collect();

    if pixels.is_empty() {
        return FNV_OFFSET;
    }

    // Walk in capture order (= reverse of PNG image order; the panel is
    // mounted upside-down so the dumper stores pixels in display orientation).
    // This must match PendingWindow::push() in decoder.rs exactly.
    // bg = first pixel seen in capture order = last pixel in image order.
    let mut iter = pixels.iter().rev();
    let bg = *iter.next().unwrap();

    let mut hash = FNV_OFFSET;
    let mut run_len: u16 = 1; // the bg pixel we just consumed
    let mut run_is_fg = false; // first run is always bg

    for &px in iter {
        let is_fg = px != bg;
        if is_fg == run_is_fg {
            run_len += 1;
        } else {
            hash = fnv_mix(hash, run_len as u8);
            hash = fnv_mix(hash, (run_len >> 8) as u8);
            run_len = 1;
            run_is_fg = is_fg;
        }
    }
    // Final run.
    hash = fnv_mix(hash, run_len as u8);
    hash = fnv_mix(hash, (run_len >> 8) as u8);
    hash
}
