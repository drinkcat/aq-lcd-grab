#!/usr/bin/env bash
# Cargo `runner` for STM32F103 over USART1 ROM bootloader via stm32flash.
#
# Wiring (USB-UART adapter -> STM32):
#   adapter TX  -> STM32 PA10 (USART1 RX)
#   adapter RX  -> STM32 PA9  (USART1 TX)
#   adapter GND -> STM32 GND
#   STM32 BOOT0 -> 3V3 (enter ROM bootloader), then pulse NRST low.
#
# After flashing, return BOOT0 low and pulse NRST again to boot user
# firmware. stm32flash's `-g 0x08000000` triggers a "go" jump instead
# of a reset (no NRST line on a plain USB-UART), but a manual reset
# after pulling BOOT0 low is the most reliable.
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

# -w: write, -v: verify, -g 0x08000000: go (jump to user code after write).
stm32flash -b "$BAUD" -w "$BIN" -v -g 0x08000000 "$PORT"
