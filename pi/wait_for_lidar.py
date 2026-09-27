#!/usr/bin/env python3
"""Verify the fixed LiDAR device path and recreate its runtime link."""

from __future__ import annotations

import argparse
from pathlib import Path

from device_discovery import link
from device_paths import ESP32_PORT, LIDAR_PORT, fixed_serial_port


def ensure_lidar_link(runtime_dir: Path) -> bool:
    if not fixed_serial_port(LIDAR_PORT, ESP32_PORT):
        return False
    runtime_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    link(runtime_dir, "lidar", LIDAR_PORT)
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-dir", required=True, type=Path)
    args = parser.parse_args()
    return 0 if ensure_lidar_link(args.runtime_dir) else 1


if __name__ == "__main__":
    raise SystemExit(main())
