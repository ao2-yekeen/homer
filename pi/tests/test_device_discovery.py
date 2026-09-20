"""Unit tests for protocol-based hardware role discovery."""

import importlib.util
import sys
from pathlib import Path

import pytest


MODULE = Path(__file__).parents[1] / "device_discovery.py"
SPEC = importlib.util.spec_from_file_location("device_discovery", MODULE)
assert SPEC and SPEC.loader
device_discovery = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = device_discovery
SPEC.loader.exec_module(device_discovery)


def test_discover_assigns_roles_independently_of_tty_order(monkeypatch):
    monkeypatch.setattr(device_discovery.glob, "glob", lambda pattern: ["/dev/v4l/by-id/c930e"] if "v4l" in pattern else [])
    monkeypatch.setattr(device_discovery.os.path, "exists", lambda path: True)
    monkeypatch.setattr(device_discovery, "rplidar_serial", lambda port, *_: "LIDAR123" if port == "/dev/ttyUSB2" else None)

    assert device_discovery.discover("/dev/v4l/by-id/camera", ["/dev/ttyUSB2", "/dev/ttyUSB9"], 1) == {
        "esp32": "/dev/ttyUSB9",
        "lidar": "/dev/ttyUSB2",
        "rgb_camera": "/dev/v4l/by-id/c930e",
        "lidar_serial": "LIDAR123",
    }


def test_discover_rejects_missing_lidar(monkeypatch):
    monkeypatch.setattr(device_discovery.glob, "glob", lambda pattern: ["/dev/v4l/by-id/c930e"] if "v4l" in pattern else [])
    monkeypatch.setattr(device_discovery.os.path, "exists", lambda path: True)
    monkeypatch.setattr(device_discovery, "rplidar_serial", lambda *_: None)

    with pytest.raises(device_discovery.DiscoveryError, match="exactly one RPLIDAR"):
        device_discovery.discover("/dev/v4l/by-id/camera", ["/dev/ttyUSB0", "/dev/ttyUSB1"], 1)
