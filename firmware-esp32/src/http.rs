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

use picoserve::response::{Content, IntoResponse};
use picoserve::routing::get;
use picoserve::{AppWithStateBuilder, Config, Router, Timeouts};

use crate::SharedFb;

/// Number of concurrent connection-handler tasks. A browser fetches the page
/// and image and may overlap refreshes, so a small pool keeps a listener free.
pub const HTTP_WORKERS: usize = 2;

/// The shared framebuffer, published once at startup via [`set_fb`].
static FB: AtomicPtr<SharedFb> = AtomicPtr::new(core::ptr::null_mut());

/// Publish the framebuffer handle for the HTTP handlers. Call once before
/// spawning the workers.
pub fn set_fb(fb: &'static SharedFb) {
    FB.store(fb as *const SharedFb as *mut SharedFb, Ordering::Release);
}

fn fb() -> &'static SharedFb {
    // Safe: the pointer is either null (handlers not yet serving) or a
    // 'static reference published by set_fb before any worker is spawned.
    unsafe { FB.load(Ordering::Acquire).as_ref().expect("fb not set") }
}

// Self-pacing refresh: request the next frame only after the current one has
// loaded (plus a short gap), so requests never stack up if the link is slow.
const INDEX_HTML: &str = "<!doctype html><meta charset=utf-8><title>aq-lcd</title>\
<style>body{background:#111;margin:0;display:grid;place-items:center;height:100vh}\
img{image-rendering:pixelated;height:96vh}</style>\
<img src=/fb.bmp id=i>\
<script>i.onload=i.onerror=()=>setTimeout(()=>i.src='/fb.bmp?'+Date.now(),500)</script>";

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
