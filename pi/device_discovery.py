#!/usr/bin/env python3
"""Create robot device links from fixed physical USB paths, without probing."""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
from pathlib import Path

from device_paths import ESP32_PORT, LIDAR_PORT, RGB_CAMERA, fixed_serial_port


def link(runtime_dir: Path, name: str, target: Path | None) -> None:
    destination = runtime_dir / name
    if target is None:
        destination.unlink(missing_ok=True)
        return
    replacement = runtime_dir / f".{name}.new"
    replacement.unlink(missing_ok=True)
    replacement.symlink_to(target)
    replacement.replace(destination)


def write_json(path: Path, payload: dict[str, str]) -> None:
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", dir=path.parent, delete=False, encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    temporary.replace(path)


def discover(runtime_dir: Path) -> dict[str, str]:
    runtime_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    lidar = LIDAR_PORT if fixed_serial_port(LIDAR_PORT, ESP32_PORT) else None
    esp32 = ESP32_PORT if fixed_serial_port(ESP32_PORT, LIDAR_PORT) else None
    camera = RGB_CAMERA if RGB_CAMERA.exists() else None
    devices = {"lidar": lidar, "esp32": esp32, "rgb_camera": camera}
    for name, target in devices.items():
        link(runtime_dir, name, target)
    present = {name: str(target) for name, target in devices.items() if target is not None}
    write_json(runtime_dir / "devices.json", present)
    return present


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-dir", required=True, type=Path)
    args = parser.parse_args()
    devices = discover(args.runtime_dir)
    if "esp32" not in devices or "rgb_camera" not in devices:
        print("core discovery failed: fixed ESP32 port or RGB camera unavailable", file=sys.stderr)
        return 1
    if "lidar" not in devices:
        print("LiDAR unavailable; fixed ESP32 and RGB links remain available", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
