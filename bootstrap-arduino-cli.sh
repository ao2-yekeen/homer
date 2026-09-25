#!/usr/bin/env bash
# Install the pinned Arduino core and micro-ROS library required by build.sh.
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ESP32_INDEX="https://espressif.github.io/arduino-esp32/package_esp32_index.json"
MICROROS_DIR="$PROJECT_DIR/.arduino/libraries/micro_ros_arduino"

if ! command -v arduino-cli >/dev/null; then
  echo "ERROR: arduino-cli is not installed. Install Arduino CLI, then re-run this script." >&2
  exit 1
fi

# Arduino-ESP32 2.0.2 invokes esptool with `python`; it needs pyserial in that
# interpreter (which may differ from the `python3` command on older systems).
if ! command -v python >/dev/null; then
  echo "ERROR: Python is required by the ESP32 Arduino core." >&2
  exit 1
fi
if ! python -c 'import serial' >/dev/null 2>&1; then
  python -m pip install --user pyserial
fi

echo "==> Installing ESP32 Arduino core 2.0.2"
arduino-cli core update-index --additional-urls "$ESP32_INDEX"
arduino-cli core install esp32:esp32@2.0.2 --additional-urls "$ESP32_INDEX"

if [ ! -d "$MICROROS_DIR/.git" ]; then
  echo "==> Fetching micro-ROS Arduino v2.0.8-jazzy"
  mkdir -p "$(dirname "$MICROROS_DIR")"
  git clone --depth 1 --branch v2.0.8-jazzy \
    https://github.com/micro-ROS/micro_ros_arduino.git "$MICROROS_DIR"
fi

echo "==> Arduino CLI dependencies are ready"
