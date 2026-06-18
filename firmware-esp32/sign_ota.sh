#!/bin/bash
# sign_ota.sh — build, sign, and OTA-upload firmware to the device.
#
# Usage:
#   ./sign_ota.sh <device-ip> [privkey.pem]
#
# privkey.pem defaults to ./privkey.pem in the firmware-esp32 directory.
#
# Key generation (one time):
#   openssl genpkey -algorithm ed25519 -out privkey.pem
#
# Extract the 32-byte raw public key for secrets.env:
#   openssl pkey -in privkey.pem -pubout -outform DER | tail -c 32 | xxd -p | tr -d '\n'
# Add OTA_PUBKEY=<hex> to secrets.env, then rebuild and reflash via USB once.
#
# Subsequent updates (no USB cable needed):
#   ./sign_ota.sh 192.168.1.42

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DEVICE="${1:?usage: $0 <device-ip> [privkey.pem]}"
PRIVKEY="${2:-${SCRIPT_DIR}/privkey.pem}"

ELF="${SCRIPT_DIR}/target/riscv32imac-unknown-none-elf/release/aq-lcd-grab-esp32"
BIN_TMP="$(mktemp /tmp/ota_fw.XXXXXX.bin)"
SIG_TMP="$(mktemp /tmp/ota_sig.XXXXXX.bin)"
SIGNED_TMP="$(mktemp /tmp/ota_signed.XXXXXX.bin)"
trap 'rm -f "$BIN_TMP" "$SIG_TMP" "$SIGNED_TMP"' EXIT

echo "Building release firmware..."
cargo build --release --manifest-path "${SCRIPT_DIR}/Cargo.toml"

echo "Converting ELF to flash binary..."
espflash save-image --chip esp32c6 --flash-freq 40mhz --partition-table "${SCRIPT_DIR}/partitions.csv" "$ELF" "$BIN_TMP"

# Ed25519 in OpenSSL only supports pure Ed25519 (not the prehashed Ed25519ph
# variant). To avoid loading the full firmware into firmware RAM for
# verify_strict(), we sign the SHA-512 hash as a 64-byte message.
# Firmware: accumulate SHA-512 while streaming, then verify_strict(hash_bytes).
HASH_TMP="$(mktemp /tmp/ota_hash.XXXXXX.bin)"
trap 'rm -f "$BIN_TMP" "$SIG_TMP" "$SIGNED_TMP" "$HASH_TMP"' EXIT
openssl dgst -sha512 -binary "$BIN_TMP" > "$HASH_TMP"
openssl pkeyutl -sign -inkey "$PRIVKEY" -in "$HASH_TMP" -out "$SIG_TMP"
cat "$BIN_TMP" "$SIG_TMP" > "$SIGNED_TMP"

FW=$(wc -c < "$BIN_TMP")
SIG=$(wc -c < "$SIG_TMP")
TOTAL=$(wc -c < "$SIGNED_TMP")
echo "Firmware: ${FW} bytes, signature: ${SIG} bytes, total: ${TOTAL} bytes"

echo "Uploading to http://${DEVICE}/ota ..."
curl -f -X POST "http://${DEVICE}/ota" \
     -H "Content-Type: application/octet-stream" \
     --data-binary "@${SIGNED_TMP}"
echo
echo "Done. Device will reboot into the new firmware."
