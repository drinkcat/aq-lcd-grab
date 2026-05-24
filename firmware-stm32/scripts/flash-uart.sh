#!/usr/bin/env bash
# Cargo `runner` for STM32F103 over USART1 ROM bootloader via stm32flash.
#
# Wiring (USB-UART adapter -> STM32):
#   adapter TX  -> STM32 PA10 (USART1 RX)
#   adapter RX  -> STM32 PA9  (USART1 TX)
#   adapter GND -> STM32 GND
#   adapter DTR -[1k]-> STM32 BOOT0 (internal pull-down holds it low at idle)
#   adapter RTS -[1k]-> STM32 NRST  (internal pull-up holds it high at idle)
#
# Override port via STM32FLASH_PORT, baud via STM32FLASH_BAUD.

set -euo pipefail

ELF="${1:-}"
if [[ -z "$ELF" ]]; then
    echo "usage: $0 <elf>" >&2
    exit 2
fi

PORT="${STM32FLASH_PORT:-/dev/ttyUSB0}"
BAUD="${STM32FLASH_BAUD:-57600}"

if ! command -v stm32flash >/dev/null; then
    echo "error: stm32flash not found in PATH" >&2
    echo "  Arch: sudo pacman -S stm32flash" >&2
    echo "  Debian/Ubuntu: sudo apt install stm32flash" >&2
    exit 1
fi

if ! command -v arm-none-eabi-objcopy >/dev/null && ! command -v llvm-objcopy >/dev/null; then
    echo "error: need arm-none-eabi-objcopy or llvm-objcopy in PATH" >&2
    exit 1
fi

BIN="${ELF}.bin"
if command -v arm-none-eabi-objcopy >/dev/null; then
    arm-none-eabi-objcopy -O binary "$ELF" "$BIN"
else
    llvm-objcopy -O binary "$ELF" "$BIN"
fi

SIZE=$(stat -c%s "$BIN")
echo "flashing $BIN ($SIZE bytes) to $PORT @ ${BAUD} baud"

# Entry sequence '-dtr,rts,-rts': drive BOOT0 high (DTR wire-low →
# inverter → BOOT0 high), pulse NRST (RTS high → reset asserted → RTS
# low → released). Chip comes up in ROM bootloader.
#
# Exit sequence 'dtr,rts,-rts': drop BOOT0 (DTR wire-high → BOOT0 low),
# then pulse NRST so the chip resets out of bootloader into user code.
# Without an explicit exit sequence, stm32flash leaves DTR at its
# entry-sequence value (low → BOOT0 high) and `-R`'s software reset
# bounces the chip right back into the bootloader.
#
# The leading '-' inverts the sense for FT232R TTL breakouts which
# don't invert internally.
stm32flash -b "$BAUD" -i '-dtr,rts,-rts:dtr,rts,-rts' -w "$BIN" -v "$PORT"
