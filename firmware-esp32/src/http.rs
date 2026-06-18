//! HTTP server (picoserve): serves the reconstructed panel framebuffer.
//!
//! `GET /` returns a tiny auto-refreshing HTML page; `GET /fb.bmp` streams the
//! framebuffer as a 24-bit BMP. picoserve gives HTTP/1.1 keep-alive and a
//! correct `Content-Length` (the BMP size is known up front), so browsers
//! render the image reliably and reuse the connection across refreshes.
//!
//! The framebuffer handle lives in a module `static` (set once at startup)
//! rather than picoserve router state: a stateless router (`State = ()`) keeps
//! the router type nameable for `static` storage without extra opaque-type
//! gymnastics.

use core::sync::atomic::{AtomicPtr, Ordering};

use picoserve::response::sse::{EventSource, EventWriter};
use picoserve::response::{Content, IntoResponse};
use picoserve::routing::get;
use picoserve::{AppWithStateBuilder, Config, Router, Timeouts};

use crate::{logger, LatestValues, SharedFb, ROW_NAMES};

/// Number of concurrent connection-handler tasks. A browser fetches the page
/// and image and may overlap refreshes, so a small pool keeps a listener free.
pub const HTTP_WORKERS: usize = 2;

/// The shared framebuffer + latest values, published once at startup via
/// [`set_shared`].
static FB: AtomicPtr<SharedFb> = AtomicPtr::new(core::ptr::null_mut());
static LATEST: AtomicPtr<LatestValues> = AtomicPtr::new(core::ptr::null_mut());

/// Publish the shared state for the HTTP handlers. Call once before spawning
/// the workers.
pub fn set_shared(fb: &'static SharedFb, latest: &'static LatestValues) {
    FB.store(fb as *const SharedFb as *mut SharedFb, Ordering::Release);
    LATEST.store(
        latest as *const LatestValues as *mut LatestValues,
        Ordering::Release,
    );
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
    }
}

/// The nameable router type for `static` storage.
pub type AppRouter = picoserve::AppRouter<AppProps>;

/// picoserve config: keep-alive on (we run a worker pool), modest timeouts.
pub fn config() -> Config {
    Config::new(Timeouts {
        start_read_request: embassy_time::Duration::from_secs(10),
        persistent_start_read_request: embassy_time::Duration::from_secs(5),
        read_request: embassy_time::Duration::from_secs(10),
        write: embassy_time::Duration::from_secs(10),
    })
    .keep_connection_alive()
}
