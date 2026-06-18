//! HTTP server (picoserve): serves the reconstructed panel framebuffer and
//! accepts OTA firmware uploads.
//!
//! `GET /` returns a tiny auto-refreshing HTML page; `GET /fb.bmp` streams the
//! framebuffer as a 24-bit BMP. picoserve gives HTTP/1.1 keep-alive and a
//! correct `Content-Length` (the BMP size is known up front), so browsers
//! render the image reliably and reuse the connection across refreshes.
//!
//! `POST /ota` accepts a firmware image with a trailing 64-byte Ed25519
//! signature. The handler streams the body in 4 KB chunks, writing directly to
//! the inactive OTA flash partition while hashing the firmware bytes. After the
//! last firmware byte it reads the 64-byte signature, verifies it against the
//! accumulated SHA-512 hash using the baked-in public key, and — on success —
//! activates the new partition and reboots.
//!
//! If verification fails the OTA data partition is left untouched, so the
//! bootloader continues booting the current slot. The corrupted bytes sitting in
//! the inactive slot are harmless: the bootloader never selects a slot without
//! an explicit entry in the OTA data partition, and the next upload overwrites
//! from offset 0 again.
//!
//! The framebuffer handle lives in a module `static` (set once at startup)
//! rather than picoserve router state: a stateless router (`State = ()`) keeps
//! the router type nameable for `static` storage without extra opaque-type
//! gymnastics.

use core::sync::atomic::{AtomicPtr, Ordering};

use ed25519_dalek::{Digest as _, Sha512, Signature, VerifyingKey};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embedded_storage::nor_flash::NorFlash as _;
use embedded_storage::ReadStorage as _;
use esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN;
use esp_storage::FlashStorage;
use log::{info, warn};
use picoserve::io::Read;
use picoserve::request::Request;
use picoserve::response::sse::{EventSource, EventWriter};
use picoserve::response::{Content, IntoResponse, ResponseWriter, StatusCode};
use picoserve::routing::{get, MethodHandlerService};
use picoserve::{AppWithStateBuilder, Config, ResponseSent, Router, Timeouts};

use crate::{logger, LatestValues, SharedFb, ROW_NAMES};

/// Number of concurrent connection-handler tasks. Four workers: one per browser
/// tab's persistent SSE stream, plus headroom for image/values requests and OTA.
pub const HTTP_WORKERS: usize = 4;

/// The shared framebuffer + latest values, published once at startup via
/// [`set_shared`].
static FB: AtomicPtr<SharedFb> = AtomicPtr::new(core::ptr::null_mut());
static LATEST: AtomicPtr<LatestValues> = AtomicPtr::new(core::ptr::null_mut());

/// Flash storage for OTA, held behind a mutex so only one OTA runs at a time.
static FLASH: Mutex<CriticalSectionRawMutex, Option<FlashStorage<'static>>> = Mutex::new(None);

/// Set to true once a valid OTA is committed; new OTA requests are rejected
/// with 503 so no second upload races the 500 ms drain + reboot.
static OTA_REBOOTING: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Chunk buffer and hasher kept in statics to avoid large stack frames in the
/// HTTP worker. Safe to access without a separate lock: only used while holding
/// the FLASH mutex above.
static mut OTA_BUF: [u8; 4096] = [0u8; 4096];
static mut OTA_HASHER: Option<Sha512> = None;

// Baked-in Ed25519 public key, written by build.rs from `secrets.env`.
include!(concat!(env!("OUT_DIR"), "/ota_pubkey.rs"));

/// Publish the shared state for the HTTP handlers. Call once before spawning
/// the workers.
pub fn set_shared(fb: &'static SharedFb, latest: &'static LatestValues) {
    FB.store(fb as *const SharedFb as *mut SharedFb, Ordering::Release);
    LATEST.store(
        latest as *const LatestValues as *mut LatestValues,
        Ordering::Release,
    );
}

/// Publish the flash storage for OTA. Call once before spawning the workers.
pub fn set_flash(flash: FlashStorage<'static>) {
    // Safe to block here: called before any HTTP workers are spawned.
    critical_section::with(|_| {
        if let Ok(mut g) = FLASH.try_lock() {
            *g = Some(flash);
        }
    });
}

fn fb() -> &'static SharedFb {
    // Safe: the pointer is either null (handlers not yet serving) or a
    // 'static reference published by set_shared before any worker is spawned.
    unsafe { FB.load(Ordering::Acquire).as_ref().expect("fb not set") }
}

fn latest() -> &'static LatestValues {
    unsafe {
        LATEST
            .load(Ordering::Acquire)
            .as_ref()
            .expect("latest not set")
    }
}

// Image self-paces (next frame only after onload + gap) so requests never
// stack; the values panel polls /values once a second and renders the JSON.
// The log panel streams via SSE (/log-stream) and appends lines in a scrolling
// <pre> (max 200 lines kept to bound DOM size).
const INDEX_HTML: &str = "<!doctype html><meta charset=utf-8><title>aq-lcd</title>\
<style>\
*{box-sizing:border-box}\
html,body{height:100%;margin:0}\
body{background:#111;color:#eee;font:16px system-ui;\
display:flex;gap:24px;align-items:stretch;justify-content:center;padding:16px;overflow:hidden}\
img{image-rendering:pixelated;height:100%;width:auto;flex-shrink:0;display:block}\
#v{display:grid;grid-template-columns:auto auto;gap:6px 16px;align-self:center}\
#v b{color:#8cf;font-weight:600}#v span{text-align:right;font-variant-numeric:tabular-nums}\
.u{color:#888;font-size:12px}\
#log-wrap{display:flex;flex-direction:column;min-width:320px;max-width:480px;flex:1;overflow:hidden}\
#log-wrap h3{margin:0 0 4px;font-size:13px;color:#8cf;flex-shrink:0}\
#log{background:#1a1a1a;border:1px solid #333;border-radius:4px;padding:8px;\
font:12px/1.4 monospace;overflow-y:auto;flex:1;white-space:pre-wrap;word-break:break-all;margin:0}\
</style>\
<img src=/fb.bmp id=i>\
<div id=v>loading…</div>\
<div id=log-wrap><h3>Console</h3><pre id=log></pre></div>\
<script>\
const U={pm25:'µg/m³',tvoc:'ppm',co2:'ppm',temp:'°C',humidity:'%'};\
i.onload=i.onerror=()=>setTimeout(()=>i.src='/fb.bmp?'+Date.now(),500);\
async function vp(){try{const d=await(await fetch('/values?'+Date.now())).json();\
v.innerHTML=Object.keys(U).map(k=>`<b>${k}</b><span>${d[k]||'–'} <i class=u>${U[k]}</i></span>`).join('')}\
catch(e){}setTimeout(vp,1000)}vp();\
const MAX_LOG=200;\
const es=new EventSource('/log-stream');\
es.addEventListener('log',e=>{\
const el=document.getElementById('log');\
el.textContent+=e.data+'\\n';\
const lines=el.textContent.split('\\n');\
if(lines.length>MAX_LOG+1)el.textContent=lines.slice(lines.length-MAX_LOG-1).join('\\n');\
el.scrollTop=el.scrollHeight;\
});\
</script>";

/// A response that streams the framebuffer as a 24-bit BMP with a known length.
struct BmpResponse;

impl Content for BmpResponse {
    fn content_type(&self) -> &'static str {
        "image/bmp"
    }

    fn content_length(&self) -> usize {
        // 4bpp palettized BMP — ~6× smaller than 24-bit (the panel uses ≤16
        // colours and the framebuffer already stores 4bpp indices).
        framebuffer::BMP4_LEN
    }

    async fn write_content<W: picoserve::io::Write>(self, mut writer: W) -> Result<(), W::Error> {
        let fb = fb().lock().await;
        let store = fb.store();
        writer
            .write_all(&framebuffer::bmp4_header(&store.palette))
            .await?;
        // Stream 4bpp pixel data in large chunks so each write fills full TCP
        // segments (small writes produced runt packets + stop-and-go ACKs).
        // 4096 pixels = 2048 bytes per write (> one 1460-MSS segment).
        const CHUNK_PX: usize = 4096;
        let mut chunk = [0u8; CHUNK_PX / 2];
        let mut start = 0;
        while start < framebuffer::PIXELS {
            let count = CHUNK_PX.min(framebuffer::PIXELS - start);
            let n = framebuffer::bmp4_pixels(store, start, count, &mut chunk);
            writer.write_all(&chunk[..n]).await?;
            start += count;
        }
        Ok(())
    }
}

/// `&'static str` served as `text/html` (picoserve's `&str` Content defaults to
/// `text/plain`, which makes the browser show the source instead of rendering).
struct Html(&'static str);

impl Content for Html {
    fn content_type(&self) -> &'static str {
        "text/html; charset=utf-8"
    }
    fn content_length(&self) -> usize {
        self.0.len()
    }
    async fn write_content<W: picoserve::io::Write>(self, mut writer: W) -> Result<(), W::Error> {
        writer.write_all(self.0.as_bytes()).await
    }
}

async fn index() -> impl IntoResponse {
    Html(INDEX_HTML)
}

async fn fb_bmp() -> impl IntoResponse {
    BmpResponse
}

/// `GET /log-stream` → SSE stream of console log lines.
///
/// The handler loops: await the logger signal, drain any buffered bytes, split
/// into lines, and emit one SSE `log` event per line. The browser reconnects
/// automatically if the connection drops.
struct LogStream;

impl EventSource for LogStream {
    async fn write_events<W: picoserve::io::Write>(
        self,
        mut writer: EventWriter<'_, W>,
    ) -> Result<(), W::Error> {
        let mut line: heapless::String<256> = heapless::String::new();
        loop {
            // Flush all complete lines already in the ring first.
            while logger::pop_line(&mut line) {
                if !line.is_empty() {
                    writer.write_event("log", line.as_str()).await?;
                }
            }
            // Sleep until the logger deposits more bytes.
            logger::SIGNAL.wait().await;
        }
    }
}

async fn log_stream() -> impl IntoResponse {
    picoserve::response::sse::EventStream(LogStream)
}

/// `GET /values` → JSON of the latest decoded values, e.g.
/// `{"pm25":"6","tvoc":"0.1","co2":"586","temp":"26","humidity":"55"}`.
/// Built into a small heapless string (values are short).
async fn values() -> impl IntoResponse {
    use core::fmt::Write as _;
    let g = latest().lock().await;
    let mut json = heapless::String::<256>::new();
    let _ = json.push('{');
    for (i, name) in ROW_NAMES.iter().enumerate() {
        if i > 0 {
            let _ = json.push(',');
        }
        // Values are decoded digits/dot/space — JSON-safe without escaping.
        let _ = write!(json, "\"{}\":\"{}\"", name, g[i].as_str());
    }
    let _ = json.push('}');
    Json(json)
}

/// A heapless JSON string served with caller-set headers.
struct Json(heapless::String<256>);

impl Content for Json {
    fn content_type(&self) -> &'static str {
        "application/json"
    }
    fn content_length(&self) -> usize {
        self.0.len()
    }
    async fn write_content<W: picoserve::io::Write>(self, mut writer: W) -> Result<(), W::Error> {
        writer.write_all(self.0.as_bytes()).await
    }
}

/// `POST /ota` service — streams the request body to the inactive OTA flash
/// partition, verifies the trailing Ed25519 signature, and reboots on success.
///
/// Implemented as a [`MethodHandlerService`] to get direct access to the
/// streaming body reader without buffering the firmware image in RAM.
struct OtaService;

impl MethodHandlerService for OtaService {
    async fn call_method_handler_service<R: Read, W: ResponseWriter<Error = R::Error>>(
        &self,
        _state: &(),
        _path_parameters: (),
        method: &str,
        mut request: Request<'_, R>,
        response_writer: W,
    ) -> Result<ResponseSent, W::Error> {
        if method != "POST" {
            return StatusCode::METHOD_NOT_ALLOWED
                .write_to(request.body_connection.finalize().await?, response_writer)
                .await;
        }

        let (status, reboot) = self.handle_ota(&mut request).await;
        let sent = status
            .write_to(request.body_connection.finalize().await?, response_writer)
            .await?;
        if reboot {
            embassy_time::Timer::after(embassy_time::Duration::from_millis(500)).await;
            esp_hal::system::software_reset();
        }
        Ok(sent)
    }
}

impl OtaService {
    async fn handle_ota<R: Read>(&self, request: &mut Request<'_, R>) -> (StatusCode, bool) {
        const SIG_LEN: usize = 64;
        const CHUNK: usize = 4096;

        if OTA_REBOOTING.load(Ordering::Acquire) {
            warn!("OTA: reboot already pending, rejecting new request");
            return (StatusCode::SERVICE_UNAVAILABLE, false);
        }

        let total = request.body_connection.content_length();
        if total <= SIG_LEN {
            warn!("OTA: body too small ({total} bytes)");
            return (StatusCode::BAD_REQUEST, false);
        }
        let firmware_len = total - SIG_LEN;
        info!("OTA: starting, firmware={firmware_len} B + {SIG_LEN}-byte sig");

        let mut flash_guard = FLASH.lock().await;
        let flash = match flash_guard.as_mut() {
            Some(f) => f,
            None => {
                warn!("OTA: flash not initialised");
                return (StatusCode::INTERNAL_SERVER_ERROR, false);
            }
        };

        // SAFETY: OTA_BUF and OTA_HASHER are only accessed while holding the
        // FLASH mutex, so no concurrent access is possible.
        let (buf, hasher_slot) = unsafe { (&mut *(&raw mut OTA_BUF), &mut *(&raw mut OTA_HASHER)) };
        *hasher_slot = Some(Sha512::new());

        let mut pt_buf = [0u8; PARTITION_TABLE_MAX_LEN];
        let mut updater =
            match esp_bootloader_esp_idf::ota_updater::OtaUpdater::new(flash, &mut pt_buf) {
                Ok(u) => u,
                Err(e) => {
                    warn!("OTA: partition error {e:?}");
                    return (StatusCode::INTERNAL_SERVER_ERROR, false);
                }
            };

        let (mut target_partition, slot) = match updater.next_partition() {
            Ok(p) => p,
            Err(e) => {
                warn!("OTA: no target partition {e:?}");
                return (StatusCode::INTERNAL_SERVER_ERROR, false);
            }
        };
        info!("OTA: writing to {slot:?}");

        let body = request.body_connection.body();
        let mut reader = body.reader();
        let mut written: usize = 0;

        while written < firmware_len {
            let want = CHUNK.min(firmware_len - written);
            if reader.read_exact(&mut buf[..want]).await.is_err() {
                warn!("OTA: read error at byte {written}");
                return (StatusCode::BAD_REQUEST, false);
            }
            hasher_slot.as_mut().unwrap().update(&buf[..want]);
            if let Err(e) = embedded_storage::Storage::write(&mut target_partition, written as u32, &buf[..want]) {
                warn!("OTA: flash write error {e:?}");
                return (StatusCode::INTERNAL_SERVER_ERROR, false);
            }
            written += want;
        }

        // Read trailing 64-byte signature.
        let mut sig_bytes = [0u8; SIG_LEN];
        if reader.read_exact(&mut sig_bytes).await.is_err() {
            warn!("OTA: failed to read signature bytes");
            return (StatusCode::BAD_REQUEST, false);
        }

        // Verify Ed25519 signature against SHA-512 hash of firmware.
        let vk = match VerifyingKey::from_bytes(&OTA_PUBKEY) {
            Ok(k) => k,
            Err(e) => {
                warn!("OTA: bad public key in firmware {e:?}");
                return (StatusCode::INTERNAL_SERVER_ERROR, false);
            }
        };
        let sig = match Signature::from_slice(&sig_bytes) {
            Ok(s) => s,
            Err(e) => {
                warn!("OTA: malformed signature {e:?}");
                return (StatusCode::BAD_REQUEST, false);
            }
        };
        // OpenSSL Ed25519 only supports pure Ed25519 (not Ed25519ph). The script
        // signs SHA-512(firmware) as a 64-byte message, so we verify the same way.
        let hash_bytes = hasher_slot.take().unwrap().finalize();
        if let Err(e) = vk.verify_strict(&hash_bytes, &sig) {
            warn!("OTA: signature verification failed {e:?}");
            // Wipe the first flash sector so a partial write can't be mistaken
            // for valid firmware if OTA data were ever corrupted.
            let _ = target_partition.erase(0, 4096);
            return (StatusCode::FORBIDDEN, false);
        }

        // Erase from end of firmware to end of partition so no stale bytes from
        // a previous (larger) OTA image remain in the inactive slot.
        // FlashRegion::erase(from, to) bounds-checks both `from` and `to` as
        // strictly inside [0, partition_size), so `to` must be < partition_size.
        // esp-storage iterates sectors [from/4096, to/4096), so passing
        // `partition_size - 1` as `to` erases one sector short of the end.
        // That last sector is negligible — the bootloader won't execute a slot
        // without an explicit otadata entry regardless.
        const ERASE_SIZE: u32 = 4096;
        let partition_size = target_partition.capacity() as u32;
        let erase_from = (firmware_len as u32 + ERASE_SIZE - 1) & !(ERASE_SIZE - 1);
        if erase_from < partition_size {
            if let Err(e) = target_partition.erase(erase_from, partition_size - 1) {
                warn!("OTA: tail erase failed {e:?} (non-fatal)");
            }
        }

        // Signature OK — activate slot and reboot.
        info!("OTA: signature OK, activating {slot:?} and rebooting");
        if let Err(e) = updater.activate_next_partition() {
            warn!("OTA: activate_next_partition failed {e:?}");
            return (StatusCode::INTERNAL_SERVER_ERROR, false);
        }
        if let Err(e) = updater
            .set_current_ota_state(esp_bootloader_esp_idf::ota::OtaImageState::New)
        {
            warn!("OTA: set_current_ota_state failed {e:?}");
            return (StatusCode::INTERNAL_SERVER_ERROR, false);
        }

        // Block any further OTA attempts before releasing the flash mutex and
        // sending the response — no second upload can sneak in during the drain.
        OTA_REBOOTING.store(true, Ordering::Release);
        (StatusCode::OK, true)
    }
}

/// App props → router via picoserve's `AppWithStateBuilder`. State is `()`
/// (the framebuffer comes from the module `static`), so the router type is
/// nameable as `picoserve::AppRouter<AppProps>` for `static` storage.
pub struct AppProps;

impl AppWithStateBuilder for AppProps {
    type State = ();
    type PathRouter = impl picoserve::routing::PathRouter;

    fn build_app(self) -> Router<Self::PathRouter> {
        Router::new()
            .route("/", get(index))
            .route("/fb.bmp", get(fb_bmp))
            .route("/values", get(values))
            .route("/log-stream", get(log_stream))
            .route_service("/ota", OtaService)
    }
}

/// The nameable router type for `static` storage.
pub type AppRouter = picoserve::AppRouter<AppProps>;

/// picoserve config: keep-alive on (we run a worker pool), OTA read timeout
/// extended to accommodate large uploads over WiFi (~900 KB at typ. ~500 KB/s
/// → ~2 s, but use 120 s to be resilient to slow connections).
pub fn config() -> Config {
    Config::new(Timeouts {
        start_read_request: embassy_time::Duration::from_secs(10),
        persistent_start_read_request: embassy_time::Duration::from_secs(5),
        read_request: embassy_time::Duration::from_secs(120),
        write: embassy_time::Duration::from_secs(30),
    })
    .keep_connection_alive()
}
