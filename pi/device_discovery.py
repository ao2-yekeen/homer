#!/usr/bin/env python3
"""Discover the Mobile Robot's USB devices by protocol, not enumeration order.

The ESP32 and RPLIDAR both use indistinguishable CP2102 USB bridges.  USB
paths, ttyUSB numbers, and the bridge serial string therefore cannot identify
them reliably after a cable or hub change. This helper identifies the
RPLIDAR with its safe GET_INFO command; with the expected two serial devices,
the other port is the ESP32. It records the resulting role-to-device map in a
user-readable file and creates per-user runtime links consumed by the ROS
services.

Run it before the serial consumers start.  It deliberately makes no ROS,
motor, or arm commands.
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
from typing import Callable

RPLIDAR_GET_INFO = b"\xa5\x50"


class DiscoveryError(RuntimeError):
    """A required device could not be identified unambiguously."""


def read_exact(fd: int, count: int, timeout_s: float) -> bytes:
    data = bytearray()
    deadline = time.monotonic() + timeout_s
    while len(data) < count:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        readable, _, _ = select.select([fd], [], [], remaining)
        if not readable:
            break
        chunk = os.read(fd, count - len(data))
        if not chunk:
            break
        data.extend(chunk)
    return bytes(data)


def configure_115200_raw(fd: int) -> None:
    settings = termios.tcgetattr(fd)
    settings[0] = termios.IGNPAR
    settings[1] = 0
    settings[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    settings[3] = 0
    settings[4] = termios.B115200
    settings[5] = termios.B115200
    settings[6][termios.VMIN] = 0
    settings[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, settings)
    termios.tcflush(fd, termios.TCIOFLUSH)


def rplidar_serial(port: str, timeout_s: float = 1.5) -> str | None:
    """Return an RPLIDAR serial number, or ``None`` if *port* is not one."""
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    original_settings = termios.tcgetattr(fd)
    try:
        configure_115200_raw(fd)
        os.write(fd, RPLIDAR_GET_INFO)
        descriptor = read_exact(fd, 7, timeout_s)
        if len(descriptor) != 7 or descriptor[:2] != b"\xa5\x5a":
            return None
        payload_size = int.from_bytes(descriptor[2:6], "little") & 0x3FFFFFFF
        if payload_size != 20 or descriptor[6] != 0x04:
            return None
        payload = read_exact(fd, payload_size, timeout_s)
        if len(payload) != payload_size:
            return None
        return payload[4:20].hex().upper()
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, original_settings)
        os.close(fd)


def discover(camera_glob: str, ports: list[str], timeout_s: float) -> dict[str, str]:
    camera = sorted(glob.glob(camera_glob))
    if len(camera) != 1:
        raise DiscoveryError(f"expected exactly one RGB camera matching {camera_glob!r}; found {camera}")
    candidates = [port for port in ports if os.path.exists(port)]
    lidars = [(port, rplidar_serial(port, timeout_s)) for port in candidates]
    lidar_ports = [(port, serial) for port, serial in lidars if serial is not None]
    if len(lidar_ports) != 1:
        raise DiscoveryError(f"expected exactly one RPLIDAR among {candidates}; found {lidar_ports}")
    lidar_port, lidar_serial = lidar_ports[0]
    esp_ports = [port for port in candidates if port != lidar_port]
    if len(esp_ports) != 1:
        raise DiscoveryError(f"expected exactly one ESP32 after identifying the RPLIDAR; found {esp_ports}")
    return {"esp32": esp_ports[0], "lidar": lidar_port, "rgb_camera": camera[0], "lidar_serial": lidar_serial}


def write_json(path: Path, devices: dict[str, str]) -> None:
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", dir=path.parent, delete=False, prefix=f".{path.name}.", encoding="utf-8") as handle:
        json.dump(devices, handle, indent=2, sort_keys=True)
        handle.write("\n")
        manifest = Path(handle.name)
    manifest.replace(path)


def write_links(runtime_dir: Path, devices: dict[str, str]) -> None:
    runtime_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    for name in ("esp32", "lidar", "rgb_camera"):
        target = devices[name]
        replacement = runtime_dir / f".{name}.new"
        replacement.unlink(missing_ok=True)
        replacement.symlink_to(target)
        replacement.replace(runtime_dir / name)
    write_json(runtime_dir / "devices.json", devices)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-dir", required=True, type=Path)
    parser.add_argument("--routes-file", type=Path, help="persistent JSON file containing final role-to-device routes")
    parser.add_argument("--camera-glob", default="/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e-video-index0")
    parser.add_argument("--port", action="append", dest="ports")
    parser.add_argument("--timeout", type=float, default=4.0)
    args = parser.parse_args(argv)
    ports = args.ports or sorted(glob.glob("/dev/ttyUSB*"))
    try:
        devices = discover(args.camera_glob, ports, args.timeout)
        write_links(args.runtime_dir, devices)
        if args.routes_file:
            write_json(args.routes_file, devices)
    except (DiscoveryError, OSError) as error:
        print(f"device discovery failed: {error}", file=sys.stderr)
        return 1
    print(json.dumps(devices, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
