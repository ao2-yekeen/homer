#!/usr/bin/env bash
# Build locally, then push the built binaries to the Pi and flash the ESP32
# attached to it (over /dev/ttyUSB0) via the Pi's lightweight `esptool`.
# Usage: ./deploy.sh [esp32dev|servo_sweep|servo_idle|motor_diagnostic]
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENVIRONMENT="${1:-esp32dev}"
BUILD_DIR="$PROJECT_DIR/.pio/build/$ENVIRONMENT"
BOOT_APP0="/home/ao2-yekeen/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin"
REMOTE_DIR="~/esp32_fw"
REMOTE_PORT="/dev/ttyUSB0"

echo "==> Building $ENVIRONMENT firmware locally"
cd "$PROJECT_DIR"
python3 -m platformio run -e "$ENVIRONMENT"

echo "==> Copying binaries to pi:$REMOTE_DIR"
ssh pi "mkdir -p $REMOTE_DIR"
scp -q \
  "$BUILD_DIR/bootloader.bin" \
  "$BUILD_DIR/partitions.bin" \
  "$BUILD_DIR/firmware.bin" \
  "$BOOT_APP0" \
  "pi:$REMOTE_DIR/"

echo "==> Releasing $REMOTE_PORT (stopping micro-ros-agent)"
ssh pi "systemctl --user stop micro-ros-agent" || true

echo "==> Flashing ESP32 over $REMOTE_PORT"
# --no-stub: the apt `esptool` package (+dfsg) ships without the precompiled
# stub-flasher blobs, so it must talk to the ROM bootloader directly.
flash_status=0
ssh pi "esptool --chip esp32 --port $REMOTE_PORT --baud 460800 --no-stub \
  --before default_reset --after hard_reset write_flash -z \
  --flash_mode dio --flash_freq 40m --flash_size 4MB \
  0x1000 $REMOTE_DIR/bootloader.bin \
  0x8000 $REMOTE_DIR/partitions.bin \
  0xe000 $REMOTE_DIR/boot_app0.bin \
  0x10000 $REMOTE_DIR/firmware.bin" || flash_status=$?

if [ "$ENVIRONMENT" = "esp32dev" ]; then
  echo "==> Reclaiming $REMOTE_PORT (restarting micro-ros-agent)"
  ssh pi "systemctl --user start micro-ros-agent"
else
  echo "==> Leaving micro-ros-agent stopped for the servo-only test"
fi

if [ "$flash_status" -ne 0 ]; then
  echo "==> Flash FAILED (exit $flash_status)"
  exit "$flash_status"
fi
echo "==> Done"
