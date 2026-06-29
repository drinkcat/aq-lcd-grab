//! Periodic reboot task: reboot ESP32 every 72 hours to clear any accumulated
//! state or memory leaks in long-running networked services.

use embassy_time::{Duration, Timer};
use log::info;

unsafe extern "C" {
    fn esp_rom_software_reset_system();
}

/// Reboot task: sleeps for 72 hours then reboots the ESP32.
#[embassy_executor::task]
pub async fn reboot_task() -> ! {
    let reboot_interval = Duration::from_secs(72 * 60 * 60);
    loop {
        Timer::after(reboot_interval).await;
        info!("72h reboot timer expired, rebooting...");
        // Trigger ESP32 software reset via ROM API.
        unsafe {
            esp_rom_software_reset_system();
        }
    }
}
