#!/usr/bin/env python3
"""Wait for the previously identified LiDAR before starting its ROS service."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from device_discovery import is_rplidar, link, load_routes


def ensure_lidar_link(routes_file: Path, runtime_dir: Path, timeout: float) -> bool:
    port = load_routes(routes_file).get("lidar")
    if not port or not os.path.exists(port):
        return False
    esp32_link = runtime_dir / "esp32"
    if esp32_link.exists() and os.path.realpath(port) == os.path.realpath(esp32_link):
        return False
    if not is_rplidar(port, timeout):
        return False
    runtime_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    link(runtime_dir, "lidar", port)
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--routes-file", required=True, type=Path)
    parser.add_argument("--runtime-dir", required=True, type=Path)
    args = parser.parse_args()
    return 0 if ensure_lidar_link(args.routes_file, args.runtime_dir, 1.5) else 1


if __name__ == "__main__":
    raise SystemExit(main())
