#!/bin/bash
# Upload SlamWalker firmware to Arduino Uno
# Requires manual reset: press RESET button when prompted

HEX_FILE="$(find ~/.cache/arduino/sketches -name 'test.ino.hex' 2>/dev/null | head -1)"
PORT="${1:-/dev/ttyACM0}"

if [ -z "$HEX_FILE" ]; then
    echo "[ERROR] Hex file not found. Run: arduino-cli compile --fqbn arduino:avr:uno ~/walker_ws/arduino/test"
    exit 1
fi

echo "============================================"
echo "  SlamWalker Arduino Firmware Upload"
echo "============================================"
echo "Port: $PORT"
echo "Hex:  $HEX_FILE"
echo ""
echo ">> Press Arduino RESET button NOW, then hit Enter within 1 second <<"
read -r

avrdude -p atmega328p -c arduino -P "$PORT" -b 115200 \
    -U flash:w:"$HEX_FILE":i

if [ $? -eq 0 ]; then
    echo ""
    echo "[OK] Firmware uploaded successfully!"
else
    echo ""
    echo "[FAIL] Upload failed. Try again: press RESET, then immediately hit Enter."
fi
