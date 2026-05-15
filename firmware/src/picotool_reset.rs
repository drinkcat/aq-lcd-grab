//! USB control-request handler that lets `picotool reboot -f -u` reboot the
//! running firmware into BOOTSEL.
//!
//! Exposes a vendor-specific interface (class `0xFF`, subclass `0x00`,
//! protocol `0x01`) — the same descriptor combo the Pico SDK uses for its
//! "RESET" interface. When picotool sees that descriptor and the device is
//! in application mode, it sends a vendor OUT request `0x01` to that
//! interface; on receipt we call `reset_to_usb_boot` and the boot ROM takes
//! over. The control transfer never gets ACKed by the host because the
//! device has already disconnected — that's expected.

use embassy_rp::rom_data;
use embassy_usb::Handler;
use embassy_usb::control::{OutResponse, Recipient, Request, RequestType};
use embassy_usb::types::InterfaceNumber;

const RESET_REQUEST_BOOTSEL: u8 = 0x01;

pub struct PicotoolHandler {
    interface: Option<InterfaceNumber>,
}

impl PicotoolHandler {
    pub fn new() -> Self {
        Self { interface: None }
    }

    pub fn set_interface(&mut self, iface: InterfaceNumber) {
        self.interface = Some(iface);
    }
}

impl Handler for PicotoolHandler {
    fn control_out(&mut self, req: Request, _data: &[u8]) -> Option<OutResponse> {
        let iface = self.interface?;

        // picotool sends the BOOTSEL request as a Class-type interface
        // request even though our descriptor is vendor-specific. The Pico
        // SDK accepts both Class and Vendor; we do the same.
        let matches = (req.request_type == RequestType::Vendor
            || req.request_type == RequestType::Class)
            && req.recipient == Recipient::Interface
            && req.index == u8::from(iface) as u16;

        if !matches {
            return None;
        }

        if req.request == RESET_REQUEST_BOOTSEL {
            // Doesn't return — the boot ROM resets and re-enumerates as BOOTSEL.
            rom_data::reset_to_usb_boot(0, 0);
        }

        Some(OutResponse::Rejected)
    }
}
