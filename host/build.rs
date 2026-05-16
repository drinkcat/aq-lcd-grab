//! Bake `templates/<W>x<H>/<label>.png` into a packed-bit table the
//! decoder can match against without I/O at runtime.
//!
//! Each template PNG is the binarized output of the dumper: any non-black
//! pixel is foreground. Output emitted to `$OUT_DIR/templates_gen.rs`:
//!
//! ```ignore
//! pub static TEMPLATES: &[Template] = &[
//!     Template { w: 40, h: 61, label: '0', mask: &[..] },
//!     ..
//! ];
//! ```

use std::path::{Path, PathBuf};

fn main() {
    let dir = PathBuf::from("templates");
    println!("cargo:rerun-if-changed={}", dir.display());

    let mut entries: Vec<(u16, u16, String, Vec<u8>)> = Vec::new();
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
            let packed = pack_png(&path, w, h);
            entries.push((w, h, stem, packed));
        }
    }

    entries.sort_by(|a, b| (a.0, a.1, &a.2).cmp(&(b.0, b.1, &b.2)));

    let out_dir = std::env::var_os("OUT_DIR").expect("OUT_DIR");
    let out_path = Path::new(&out_dir).join("templates_gen.rs");
    let mut src = String::new();
    src.push_str("pub struct Template { pub w: u16, pub h: u16, pub label: &'static str, pub mask: &'static [u8] }\n");
    src.push_str("pub static TEMPLATES: &[Template] = &[\n");
    for (w, h, label, packed) in &entries {
        src.push_str(&format!(
            "    Template {{ w: {w}, h: {h}, label: {label:?}, mask: &{packed:?} }},\n"
        ));
    }
    src.push_str("];\n");
    std::fs::write(&out_path, src).expect("write templates_gen.rs");
}

fn parse_size(s: &str) -> Option<(u16, u16)> {
    let (w, h) = s.split_once('x')?;
    Some((w.parse().ok()?, h.parse().ok()?))
}

fn pack_png(path: &Path, expected_w: u16, expected_h: u16) -> Vec<u8> {
    let img = image::open(path).unwrap_or_else(|e| panic!("open {}: {e}", path.display()));
    let img = img.to_luma8();
    let (w, h) = (img.width() as u16, img.height() as u16);
    assert_eq!(
        (w, h),
        (expected_w, expected_h),
        "{}: dim {w}x{h} != dir {expected_w}x{expected_h}",
        path.display()
    );
    let n = w as usize * h as usize;
    let mut out = vec![0u8; (n + 7) / 8];
    for (i, px) in img.pixels().enumerate() {
        if px.0[0] != 0 {
            out[i / 8] |= 1 << (i % 8);
        }
    }
    out
}
