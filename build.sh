#!/usr/bin/env bash
# Build an Arduino CLI firmware image.
# Usage: ./build.sh [esp32dev|servo_sweep|servo_idle|motor_diagnostic|encoder_calibration|micro_ros_serial]
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="${1:-esp32dev}"

case "$TARGET" in
  esp32dev)
    SKETCH="$PROJECT_DIR/firmware"
    PROFILE="production"
    ;;
  servo_sweep|servo_idle|motor_diagnostic|encoder_calibration|micro_ros_serial)
    SKETCH="$PROJECT_DIR/diagnostics/$TARGET"
    PROFILE="diagnostic"
    ;;
  *)
    echo "ERROR: unknown target '$TARGET'" >&2
    exit 2
    ;;
esac

if ! command -v arduino-cli >/dev/null; then
  echo "ERROR: arduino-cli is not installed. Run ./bootstrap-arduino-cli.sh after installing it." >&2
  exit 1
fi

OUTPUT_DIR="$PROJECT_DIR/build/$TARGET"
mkdir -p "$OUTPUT_DIR"

if [ "$TARGET" = "encoder_calibration" ]; then
  arduino-cli compile --clean --profile "$PROFILE" \
    --build-property "compiler.cpp.extra_flags=-I$PROJECT_DIR/firmware/src/config" \
    --output-dir "$OUTPUT_DIR" "$SKETCH"
else
  arduino-cli compile --clean --profile "$PROFILE" --output-dir "$OUTPUT_DIR" "$SKETCH"
fi
