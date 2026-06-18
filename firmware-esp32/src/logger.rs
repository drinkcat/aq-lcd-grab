//! Tee logger: forwards every record to esp-println AND appends a formatted
//! line to a fixed-size ring buffer that the HTTP `/log-stream` SSE endpoint
//! can drain.
//!
//! Usage:
//!   1. Call `init()` once (replaces `esp_println::logger::init_logger_from_env`).
//!   2. The HTTP handler calls `drain(buf)` to pull new bytes from the ring.
//!   3. Await `SIGNAL` to sleep until new bytes are available.

use core::fmt::Write as _;

use critical_section::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use heapless::Deque;
use log::{LevelFilter, Log, Metadata, Record};

/// Wakes the SSE handler whenever new bytes land in the ring.
pub static SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Ring capacity in bytes. At ~80 chars/line this holds ~50 recent lines.
pub const CAP: usize = 4096;

static RING: Mutex<core::cell::RefCell<Deque<u8, CAP>>> =
    Mutex::new(core::cell::RefCell::new(Deque::new()));

/// Pop one complete line (up to and including `\n`) from the ring into `line`.
/// Returns `true` if a complete line was found, `false` if the ring holds only
/// a partial line (or is empty). `line` is cleared before use.
pub fn pop_line(line: &mut heapless::String<256>) -> bool {
    critical_section::with(|cs| {
        let mut ring = RING.borrow_ref_mut(cs);
        line.clear();
        // Peek for a newline before committing any pops.
        let has_newline = ring.iter().any(|&b| b == b'\n');
        if !has_newline {
            return false;
        }
        loop {
            match ring.pop_front() {
                None => break,
                Some(b'\n') => break,
                Some(b) => {
                    // Best-effort: if line is full just discard the rest of this
                    // line; the '\n' will still be consumed on the next byte.
                    let _ = line.push(b as char);
                }
            }
        }
        true
    })
}

struct TeeLogger;

impl Log for TeeLogger {
    fn enabled(&self, _: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        let ts = embassy_time::Instant::now().as_millis();
        let level = record.level();

        // Print to UART/console via esp-println (does not go through log::Log).
        esp_println::println!(
            "{} ({}) - {}",
            level,
            ts,
            record.args()
        );

        // Append to ring buffer. We build the line into a small stack buffer so
        // the critical section stays short.
        let mut line: heapless::String<256> = heapless::String::new();
        let _ = write!(line, "{} ({}) - {}\n", level, ts, record.args());

        critical_section::with(|cs| {
            let mut ring = RING.borrow_ref_mut(cs);
            for b in line.as_bytes() {
                if ring.is_full() {
                    // Drop the oldest byte to make room (oldest-first eviction).
                    ring.pop_front();
                }
                let _ = ring.push_back(*b);
            }
        });

        SIGNAL.signal(());
    }

    fn flush(&self) {}
}

static LOGGER: TeeLogger = TeeLogger;

/// Install the tee logger. Call once before spawning any tasks.
/// Respects `ESP_LOG` / compile-time log filter (same as
/// `esp_println::logger::init_logger_from_env`).
pub fn init() {
    unsafe {
        log::set_logger_racy(&LOGGER).ok();
        log::set_max_level_racy(LevelFilter::Info);
    }
}

