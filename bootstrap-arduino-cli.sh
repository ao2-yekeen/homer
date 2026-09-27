#!/usr/bin/env bash
# Install the Arduino core required by build.sh.
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ESP32_INDEX="https://espressif.github.io/arduino-esp32/package_esp32_index.json"

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

echo "==> Arduino CLI dependencies are ready"
