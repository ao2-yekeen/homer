#!/usr/bin/env bash
# Build locally with Arduino CLI, then push the built binaries to the Pi and flash the ESP32
# selected by the Pi's device-discovery service via its lightweight `esptool`.
# Usage: ./deploy.sh [esp32dev|servo_sweep|servo_idle|motor_diagnostic|micro_ros_serial]
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENVIRONMENT="${1:-esp32dev}"
BUILD_DIR="$PROJECT_DIR/build/$ENVIRONMENT"
PI_UID="$(ssh pi id -u)"
REMOTE_PORT="/run/user/$PI_UID/mobile-robot-devices/esp32"
# Resolve the remote path explicitly because scp does not consistently expand
# a leading '~' in its destination argument.
REMOTE_HOME="$(ssh pi 'printf %s "$HOME"')"
REMOTE_DIR="$REMOTE_HOME/esp32_fw"
# Keep the original deployment baud rate. The value can be overridden for
# diagnostics with ESP32_FLASH_BAUD.
FLASH_BAUD="${ESP32_FLASH_BAUD:-115200}"

echo "==> Building $ENVIRONMENT firmware locally"
cd "$PROJECT_DIR"
./build.sh "$ENVIRONMENT"

case "$ENVIRONMENT" in
  esp32dev) SKETCH_NAME="firmware" ;;
  servo_sweep|servo_idle|motor_diagnostic|micro_ros_serial) SKETCH_NAME="$ENVIRONMENT" ;;
  *)
    echo "ERROR: unknown target '$ENVIRONMENT'" >&2
    exit 2
    ;;
esac

ARTIFACT_PREFIX="$BUILD_DIR/$SKETCH_NAME.ino"
BOOT_APP0="$(find "${ARDUINO_DATA_DIR:-$HOME/.arduino15}" -path '*/tools/partitions/boot_app0.bin' -type f | head -n 1)"
if [ -z "$BOOT_APP0" ]; then
  echo "ERROR: Arduino ESP32 boot_app0.bin was not found after building." >&2
  exit 1
fi
   
echo "==> Copying binaries to pi:$REMOTE_DIR"
ssh pi "mkdir -p $REMOTE_DIR"
scp -q "$ARTIFACT_PREFIX.bootloader.bin" \
  "pi:$REMOTE_DIR/bootloader.bin"
scp -q "$ARTIFACT_PREFIX.partitions.bin" \
  "pi:$REMOTE_DIR/partitions.bin"
scp -q "$ARTIFACT_PREFIX.bin" \
  "pi:$REMOTE_DIR/firmware.bin"
scp -q "$BOOT_APP0" \
  "pi:$REMOTE_DIR/boot_app0.bin"

RESTORE_AGENT=1
restore_agent() {
  if { [ "$ENVIRONMENT" = "esp32dev" ] || [ "$ENVIRONMENT" = "micro_ros_serial" ]; } &&
      [ "$RESTORE_AGENT" = 1 ]; then
    ssh pi "systemctl --user start micro-ros-agent.service" || true
  fi
}
trap restore_agent EXIT

echo "==> Releasing $REMOTE_PORT (stopping micro-ros-agent)"
ssh pi "set -e
  test -c $REMOTE_PORT
  systemctl --user stop micro-ros-agent.service
  for attempt in {1..10}; do
    if ! fuser -s $REMOTE_PORT; then
      exit 0
    fi
    sleep 1
  done
  echo 'ERROR: $REMOTE_PORT is still in use after stopping micro-ros-agent.' >&2
  fuser -v $REMOTE_PORT >&2 || true
  exit 1"

echo "==> Confirming the discovered port is an ESP32"
ssh pi "esptool --chip esp32 --port $REMOTE_PORT --baud 115200 --no-stub --after hard_reset chip_id"

echo "==> Flashing ESP32 over $REMOTE_PORT"
# --no-stub: the apt `esptool` package (+dfsg) ships without the precompiled
# stub-flasher blobs, so it must talk to the ROM bootloader directly.
ssh pi "esptool --chip esp32 --port $REMOTE_PORT --baud $FLASH_BAUD --no-stub \
  --before default_reset --after hard_reset write_flash \
  --flash_mode dio --flash_freq 40m --flash_size 4MB \
  0x1000 $REMOTE_DIR/bootloader.bin \
  0x8000 $REMOTE_DIR/partitions.bin \
  0xe000 $REMOTE_DIR/boot_app0.bin \
  0x10000 $REMOTE_DIR/firmware.bin"

if [ "$ENVIRONMENT" = "esp32dev" ] || [ "$ENVIRONMENT" = "micro_ros_serial" ]; then
  echo "==> Reclaiming $REMOTE_PORT (restarting micro-ros-agent)"
  ssh pi "systemctl --user start micro-ros-agent"
  RESTORE_AGENT=0
else
  echo "==> Leaving micro-ros-agent stopped for the standalone diagnostic"
fi
echo "==> Done"
