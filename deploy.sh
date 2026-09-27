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
FLASH_BAUD="${ESP32_FLASH_BAUD:-460800}"

echo "==> Building $ENVIRONMENT firmware locally"
cd "$PROJECT_DIR"
./build.sh "$ENVIRONMENT"

case "$ENVIRONMENT" in
  esp32dev)
    SKETCH_NAME="firmware"
    PROFILE="production"
    SKETCH_DIR="$PROJECT_DIR/firmware"
    ;;
  servo_sweep|servo_idle|motor_diagnostic|micro_ros_serial)
    SKETCH_NAME="$ENVIRONMENT"
    PROFILE="diagnostic"
    SKETCH_DIR="$PROJECT_DIR/diagnostics/$ENVIRONMENT"
    ;;
  *)
    echo "ERROR: unknown target '$ENVIRONMENT'" >&2
    exit 2
    ;;
esac

ARTIFACT_PREFIX="$BUILD_DIR/$SKETCH_NAME.ino"
CORE_DIR="$(arduino-cli compile --profile "$PROFILE" --show-properties "$SKETCH_DIR" | sed -n 's/^runtime.platform.path=//p' | head -n 1)"
BOOT_APP0="$CORE_DIR/tools/partitions/boot_app0.bin"
if [ ! -f "$BOOT_APP0" ]; then
  echo "ERROR: matching Arduino ESP32 boot_app0.bin was not found." >&2
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
if [ "$ENVIRONMENT" = "esp32dev" ]; then
  scp -q "$PROJECT_DIR/pi/micro-ros-agent.service" \
    "pi:$REMOTE_DIR/micro-ros-agent.service"
  ssh pi "install -m 0644 $REMOTE_DIR/micro-ros-agent.service \$HOME/.config/systemd/user/micro-ros-agent.service && systemctl --user daemon-reload"
fi

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

echo "==> Verifying flashed image"
ssh pi "esptool --chip esp32 --port $REMOTE_PORT --baud $FLASH_BAUD --no-stub \
  --before default_reset --after hard_reset verify_flash \
  0x1000 $REMOTE_DIR/bootloader.bin \
  0x8000 $REMOTE_DIR/partitions.bin \
  0xe000 $REMOTE_DIR/boot_app0.bin \
  0x10000 $REMOTE_DIR/firmware.bin"

if [ "$ENVIRONMENT" = "esp32dev" ] || [ "$ENVIRONMENT" = "micro_ros_serial" ]; then
  echo "==> Reclaiming $REMOTE_PORT (restarting micro-ros-agent)"
  ssh pi "systemctl --user start micro-ros-agent"
  RESTORE_AGENT=0
  if [ "$ENVIRONMENT" = "esp32dev" ]; then
    echo "==> Checking ESP32 ROS messages"
    ssh pi "set -e
      source /opt/ros/jazzy/setup.bash
      timeout 15 ros2 topic echo /odom --once --qos-reliability reliable >/dev/null
      timeout 15 ros2 topic echo /drive/status --once --qos-reliability best_effort >/dev/null"
  fi
else
  echo "==> Leaving micro-ros-agent stopped for the standalone diagnostic"
fi
echo "==> Done"
