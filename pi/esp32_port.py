"""Find the ESP32 by its HOM1 greeting and maintain its runtime device link."""

from __future__ import annotations

from pathlib import Path

from device_paths import ESP32_PORT, LIDAR_PORT, fixed_serial_port


def candidates(runtime_link: Path) -> list[str]:
    # Never search /dev/ttyUSB*: opening the LiDAR port can stop its motor.
    if not fixed_serial_port(ESP32_PORT, LIDAR_PORT):
        return []
    if not runtime_link.is_char_device() or runtime_link.resolve() != ESP32_PORT.resolve():
        return []
    return [str(runtime_link)]
