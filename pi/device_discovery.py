#!/usr/bin/env python3
"""Create independent runtime links for the robot's USB devices.

The ESP32, RGB camera, and LiDAR must not share a failure domain.  A verified
route is retained for the ESP32 and camera; the LiDAR is probed separately and
its link is omitted while it is unavailable.  Services can therefore require
only the link they actually consume.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import select
import sys
import tempfile
import termios
import time
from pathlib import Path

RPLIDAR_GET_INFO = b"\xa5\x50"


def read_exact(fd: int, count: int, timeout_s: float) -> bytes:
    data = bytearray()
    deadline = time.monotonic() + timeout_s
    while len(data) < count:
        readable, _, _ = select.select([fd], [], [], max(0.0, deadline - time.monotonic()))
        if not readable:
            break
        data.extend(os.read(fd, count - len(data)))
    return bytes(data)


def write_all(fd: int, payload: bytes, timeout_s: float) -> bool:
    view = memoryview(payload)
    deadline = time.monotonic() + timeout_s
    while view:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        _, writable, _ = select.select([], [fd], [], remaining)
        if not writable:
            return False
        try:
            written = os.write(fd, view)
        except BlockingIOError:
            continue
        if written <= 0:
            return False
        view = view[written:]
    return True


def is_rplidar(port: str, timeout_s: float) -> bool:
    """Probe the RPLIDAR GET_INFO command without starting a scan."""
    fd = None
    original = None
    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        original = termios.tcgetattr(fd)
        settings = termios.tcgetattr(fd)
        settings[0:4] = [termios.IGNPAR, 0, termios.CS8 | termios.CREAD | termios.CLOCAL, 0]
        settings[4] = termios.B115200
        settings[5] = termios.B115200
        settings[6][termios.VMIN] = settings[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        termios.tcflush(fd, termios.TCIOFLUSH)
        if not write_all(fd, RPLIDAR_GET_INFO, timeout_s):
            return False
        descriptor = read_exact(fd, 7, timeout_s)
        if len(descriptor) != 7 or descriptor[:2] != b"\xa5\x5a":
            return False
        return (int.from_bytes(descriptor[2:6], "little") & 0x3FFFFFFF) == 20 and descriptor[6] == 4 and len(read_exact(fd, 20, timeout_s)) == 20
    except OSError as error:
        print(f"LiDAR probe failed on {port}: {error}", file=sys.stderr)
        return False
    finally:
        if fd is not None:
            if original is not None:
                termios.tcsetattr(fd, termios.TCSANOW, original)
            os.close(fd)


def load_routes(path: Path) -> dict[str, str]:
    try:
        with path.open(encoding="utf-8") as stream:
            routes = json.load(stream)
    except (OSError, json.JSONDecodeError):
        return {}
    return {key: str(value) for key, value in routes.items()}


def write_json(path: Path, payload: dict[str, str]) -> None:
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", dir=path.parent, delete=False, encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    temporary.replace(path)


def link(runtime_dir: Path, name: str, target: str | None) -> None:
    destination = runtime_dir / name
    if target is None:
        destination.unlink(missing_ok=True)
        return
    replacement = runtime_dir / f".{name}.new"
    replacement.unlink(missing_ok=True)
    replacement.symlink_to(target)
    replacement.replace(destination)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runtime-dir", required=True, type=Path)
    parser.add_argument("--routes-file", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=1.5)
    args = parser.parse_args()
    routes = load_routes(args.routes_file)
    args.runtime_dir.mkdir(mode=0o700, parents=True, exist_ok=True)

    # A responding LiDAR is authoritative: the remaining CP2102 is the ESP32.
    ports = sorted(glob.glob("/dev/ttyUSB*"))
    lidar_ports = [port for port in ports if is_rplidar(port, args.timeout)]
    lidar = lidar_ports[0] if len(lidar_ports) == 1 else None
    if lidar:
        others = [port for port in ports if port != lidar]
        if len(others) == 1:
            routes["esp32"] = others[0]
            routes["lidar"] = lidar

    # On a temporary LiDAR outage, retain only a still-present previously
    # verified ESP32 route. Never guess from USB enumeration order.
    esp32 = routes.get("esp32") if os.path.exists(routes.get("esp32", "")) else None
    camera = routes.get("rgb_camera")
    if not camera or not os.path.exists(camera):
        cameras = glob.glob("/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e-video-index0")
        camera = cameras[0] if len(cameras) == 1 else None
    link(args.runtime_dir, "esp32", esp32)
    link(args.runtime_dir, "rgb_camera", camera)
    link(args.runtime_dir, "lidar", lidar)
    write_json(args.runtime_dir / "devices.json", {key: value for key, value in {"esp32": esp32, "rgb_camera": camera, "lidar": lidar}.items() if value})
    if lidar and esp32 and camera:
        routes["rgb_camera"] = camera
        write_json(args.routes_file, routes)
    if not esp32 or not camera:
        print("core discovery failed: no verified ESP32 route or RGB camera", file=sys.stderr)
        return 1
    if not lidar:
        print("LiDAR unavailable; ESP32 and RGB camera links remain available", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
