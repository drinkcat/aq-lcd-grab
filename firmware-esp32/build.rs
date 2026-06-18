fn main() {
    load_secrets();
    emit_git_commit();
    emit_build_time();
    linker_be_nice();
    // make sure linkall.x is the last linker script (otherwise might cause
    // problems with flip-link)
    println!("cargo:rustc-link-arg=-Tlinkall.x");
}

/// Emit GIT_COMMIT (short hash + optional "+dirty" suffix).
/// Reruns when HEAD or the index changes; falls back to "unknown" outside git.
fn emit_git_commit() {
    // Rerun when commits or staged changes happen.
    println!("cargo:rerun-if-changed=../../.git/HEAD");
    println!("cargo:rerun-if-changed=../../.git/index");

    let commit = std::process::Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map(|s| s.trim().to_string())
        .unwrap_or_else(|| "unknown".to_string());

    let dirty = std::process::Command::new("git")
        .args(["diff", "--quiet", "HEAD"])
        .status()
        .map(|s| !s.success())
        .unwrap_or(false);

    let full = if dirty {
        format!("{commit}+dirty")
    } else {
        commit
    };

    println!("cargo:rustc-env=GIT_COMMIT={full}");
}

fn emit_build_time() {
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap();
    let secs = now.as_secs();
    // Format as "YYYY-MM-DD HH:MM:SS UTC" without any external crate.
    let s = secs % 60;
    let m = (secs / 60) % 60;
    let h = (secs / 3600) % 24;
    let days = secs / 86400; // days since 1970-01-01
    // Gregorian calendar computation.
    let (y, mo, d) = days_to_ymd(days);
    println!("cargo:rustc-env=BUILD_TIMESTAMP={y:04}-{mo:02}-{d:02} {h:02}:{m:02}:{s:02} UTC");
}

fn days_to_ymd(days: u64) -> (u64, u64, u64) {
    // Algorithm from http://howardhinnant.github.io/date_algorithms.html
    let z = days + 719468;
    let era = z / 146097;
    let doe = z % 146097;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let mo = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if mo <= 2 { y + 1 } else { y };
    (y, mo, d)
}

/// Parse `secrets.env` (a gitignored `KEY=value` file) and expose the values
/// to the firmware via `env!()` at compile time. Missing keys become empty
/// strings so the crate still builds (it just won't connect).
fn load_secrets() {
    use std::collections::HashMap;

    println!("cargo:rerun-if-changed=secrets.env");

    let vars: HashMap<String, String> = std::fs::read_to_string("secrets.env")
        .unwrap_or_default()
        .lines()
        .filter(|l| !l.trim_start().starts_with('#') && l.contains('='))
        .filter_map(|l| {
            let (k, v) = l.split_once('=')?;
            Some((k.trim().to_string(), v.trim().to_string()))
        })
        .collect();

    for var in ["WIFI_SSID", "WIFI_PASSWORD", "HA_HOST", "HA_USER", "HA_TOKEN"] {
        println!(
            "cargo:rustc-env={var}={}",
            vars.get(var).map(String::as_str).unwrap_or_default()
        );
    }

    // OTA_PUBKEY: 32-byte Ed25519 public key as a lowercase hex string (64 chars).
    // Write a Rust source file into OUT_DIR that the firmware includes verbatim.
    let pubkey_hex = vars
        .get("OTA_PUBKEY")
        .map(String::as_str)
        .unwrap_or_default();
    let pubkey_bytes: Vec<u8> = (0..pubkey_hex.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&pubkey_hex[i..i + 2], 16).unwrap_or(0))
        .collect();
    // Pad to 32 bytes if missing/empty (signature verification will just fail).
    let mut key32 = [0u8; 32];
    let n = pubkey_bytes.len().min(32);
    key32[..n].copy_from_slice(&pubkey_bytes[..n]);
    let array_body: String = key32.iter().map(|b| b.to_string()).collect::<Vec<_>>().join(", ");
    let out_dir = std::env::var("OUT_DIR").unwrap();
    std::fs::write(
        format!("{out_dir}/ota_pubkey.rs"),
        format!("pub const OTA_PUBKEY: [u8; 32] = [{array_body}];\n"),
    )
    .unwrap();
}

fn linker_be_nice() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() > 1 {
        let kind = &args[1];
        let what = &args[2];

        match kind.as_str() {
            "undefined-symbol" => match what.as_str() {
                what if what.starts_with("_defmt_") => {
                    eprintln!();
                    eprintln!(
                        "💡 `defmt` not found - make sure `defmt.x` is added as a linker script and you have included `use defmt_rtt as _;`"
                    );
                    eprintln!();
                }
                "_stack_start" => {
                    eprintln!();
                    eprintln!("💡 Is the linker script `linkall.x` missing?");
                    eprintln!();
                }
                what if what.starts_with("esp_rtos_") => {
                    eprintln!();
                    eprintln!(
                        "💡 `esp-radio` has no scheduler enabled. Make sure you have initialized `esp-rtos` or provided an external scheduler."
                    );
                    eprintln!();
                }
                "embedded_test_linker_file_not_added_to_rustflags" => {
                    eprintln!();
                    eprintln!(
                        "💡 `embedded-test` not found - make sure `embedded-test.x` is added as a linker script for tests"
                    );
                    eprintln!();
                }
                "free" | "malloc" | "calloc" | "get_free_internal_heap_size"
                | "malloc_internal" | "realloc_internal" | "calloc_internal"
                | "free_internal" => {
                    eprintln!();
                    eprintln!(
                        "💡 Did you forget the `esp-alloc` dependency or didn't enable the `compat` feature on it?"
                    );
                    eprintln!();
                }
                _ => (),
            },
            // we don't have anything helpful for "missing-lib" yet
            _ => {
                std::process::exit(1);
            }
        }

        std::process::exit(0);
    }

    println!(
        "cargo:rustc-link-arg=--error-handling-script={}",
        std::env::current_exe().unwrap().display()
    );
}
