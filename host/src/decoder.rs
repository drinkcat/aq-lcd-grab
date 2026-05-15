//! ILI9488/ST7796 protocol decoder.
//!
//! Splits a stream of bus samples into transactions: one command byte
//! (DC=0) followed by zero or more data words (DC=1). CS going high
//! ends the current transaction. Within a transaction, consecutive DC=1
//! samples are accumulated as the command's payload.

use crate::sample::Sample;

#[derive(Clone, Debug)]
pub struct Transaction {
    pub cmd: u8,
    pub data: Vec<u16>,
}

#[derive(Default)]
pub struct Decoder {
    state: State,
}

#[derive(Default)]
enum State {
    #[default]
    Idle,
    /// We saw a command byte and are now collecting data words until CS rises.
    InTransaction { tx: Transaction },
}

impl Decoder {
    /// Simple boundary rule: DC=0 starts a new command (8080 spec — the
    /// MCU pulls DC low only for command bytes). We don't use CS for
    /// framing because real captures show occasional CS=1 glitches
    /// mid-transfer that aren't transaction boundaries.
    pub fn feed(&mut self, s: Sample) -> Option<Transaction> {
        match core::mem::take(&mut self.state) {
            State::Idle => {
                if !s.dc {
                    self.state = State::InTransaction {
                        tx: Transaction {
                            cmd: (s.data & 0xFF) as u8,
                            data: Vec::new(),
                        },
                    };
                }
                None
            }
            State::InTransaction { mut tx } => {
                if s.dc {
                    // Data word for the current command.
                    tx.data.push(s.data);
                    self.state = State::InTransaction { tx };
                    None
                } else {
                    // DC=0 starts a new command — emit the previous tx.
                    let new_tx = Transaction {
                        cmd: (s.data & 0xFF) as u8,
                        data: Vec::new(),
                    };
                    self.state = State::InTransaction { tx: new_tx };
                    Some(tx)
                }
            }
        }
    }
}

/// ILI9488 / ST7796 command names — enough to make the dump readable.
pub fn cmd_name(cmd: u8) -> &'static str {
    match cmd {
        0x00 => "NOP",
        0x01 => "SOFT_RESET",
        0x04 => "READ_DISPLAY_ID",
        0x09 => "READ_DISPLAY_STATUS",
        0x10 => "SLEEP_IN",
        0x11 => "SLEEP_OUT",
        0x12 => "PARTIAL_MODE_ON",
        0x13 => "NORMAL_MODE_ON",
        0x20 => "DISPLAY_INVERSION_OFF",
        0x21 => "DISPLAY_INVERSION_ON",
        0x28 => "DISPLAY_OFF",
        0x29 => "DISPLAY_ON",
        0x2A => "SET_COLUMN_ADDRESS",
        0x2B => "SET_ROW_ADDRESS",
        0x2C => "MEMORY_WRITE",
        0x2E => "MEMORY_READ",
        0x30 => "PARTIAL_AREA",
        0x33 => "VERTICAL_SCROLLING",
        0x36 => "MEMORY_ACCESS_CONTROL",
        0x37 => "VERTICAL_SCROLL_START",
        0x38 => "IDLE_MODE_OFF",
        0x39 => "IDLE_MODE_ON",
        0x3A => "PIXEL_FORMAT_SET",
        0x3C => "MEMORY_WRITE_CONTINUE",
        0xB0 => "INTERFACE_MODE_CONTROL",
        0xB1 => "FRAME_RATE_NORMAL",
        0xB4 => "DISPLAY_INVERSION_CONTROL",
        0xB6 => "DISPLAY_FUNCTION_CONTROL",
        0xB7 => "ENTRY_MODE_SET",
        0xC0 => "POWER_CONTROL_1",
        0xC1 => "POWER_CONTROL_2",
        0xC5 => "VCOM_CONTROL",
        0xE0 => "POSITIVE_GAMMA",
        0xE1 => "NEGATIVE_GAMMA",
        0xF7 => "ADJUST_CONTROL_3",
        _ => "?",
    }
}
