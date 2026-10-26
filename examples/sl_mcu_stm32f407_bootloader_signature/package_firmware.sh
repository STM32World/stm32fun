#!/usr/bin/env bash
set -euo pipefail

# Configurable paths
FIRMWARE_BIN="${1:-app.bin}"
PRIVATE_KEY="${2:-private_key.pem}"
OUTPUT_BIN="${3:-app_signed.bin}"

# Check required files
if [[ ! -f "$FIRMWARE_BIN" ]] || [[ ! -f "$PRIVATE_KEY" ]]; then
    echo "Usage: $0 <firmware.bin> <private_key.pem> [output.bin]"
    exit 1
fi

TEMP_DIR=$(mktemp -d)
trap 'rm -rf "$TEMP_DIR"' EXIT

echo "[1/4] Calculating SHA-256 hash and ECDSA signature..."
# 1. Output DER signature from OpenSSL
openssl dgst -sha256 -sign "$PRIVATE_KEY" -out "$TEMP_DIR/sig.der" "$FIRMWARE_BIN"

# 2. Extract raw R and S components from DER signature into raw 64-byte binary
openssl asn1parse -in "$TEMP_DIR/sig.der" -inform DER > "$TEMP_DIR/sig.asn1"

# Extract hex for integer R and S
R_HEX=$(openssl asn1parse -in "$TEMP_DIR/sig.der" -inform DER | awk -F':' '/INTEGER/ {print $4}' | sed -n '1p')
S_HEX=$(openssl asn1parse -in "$TEMP_DIR/sig.der" -inform DER | awk -F':' '/INTEGER/ {print $4}' | sed -n '2p')

# Pad or strip lead zeroes so each is exactly 32 bytes (64 hex characters)
R_HEX=$(printf "%064s" "${R_HEX#00}" | tr ' ' '0' | tail -c 64)
S_HEX=$(printf "%064s" "${S_HEX#00}" | tr ' ' '0' | tail -c 64)

# Convert R + S hex to raw binary (64 bytes)
echo -n "${R_HEX}${S_HEX}" | xxd -r -p > "$TEMP_DIR/sig_raw.bin"

echo "[2/4] Building 128-byte Header..."
# Magic Bytes: "KAPP" (0x4150504B in Little-Endian)
printf '\x4B\x50\x50\x41' > "$TEMP_DIR/header.bin"

# Payload Length: 4-byte Little-Endian uint32
PAYLOAD_SIZE=$(stat -c%s "$FIRMWARE_BIN" 2>/dev/null || stat -f%z "$FIRMWARE_BIN")
python3 -c "import struct, sys; sys.stdout.buffer.write(struct.pack('<I', $PAYLOAD_SIZE))" >> "$TEMP_DIR/header.bin"

# Append 64-byte Raw Signature
cat "$TEMP_DIR/sig_raw.bin" >> "$TEMP_DIR/header.bin"

# Append 56-byte zero padding to complete the 128-byte header
dd if=/dev/zero bs=1 count=56 2>/dev/null >> "$TEMP_DIR/header.bin"

echo "[3/4] Concatenating Header and Firmware Payload..."
cat "$TEMP_DIR/header.bin" "$FIRMWARE_BIN" > "$OUTPUT_BIN"

echo "[4/4] Done!"
echo "Generated Signed Firmware : $OUTPUT_BIN"
echo "Original Payload Size     : $PAYLOAD_SIZE bytes"
echo "Final Image Size          : $(stat -c%s "$OUTPUT_BIN" 2>/dev/null || stat -f%z "$OUTPUT_BIN") bytes"
