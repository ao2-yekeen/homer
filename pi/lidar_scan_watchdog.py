#!/usr/bin/env python3
"""Watch /scan without touching the LiDAR serial port; save failure evidence."""

from __future__ import annotations

import glob
import json
import logging
import os
import queue
import subprocess
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

SCAN_TIMEOUT = 15.0
STARTUP_GRACE = 60.0
REPEAT_INTERVAL = 60.0
MAX_SNAPSHOTS = 30
MAX_OUTPUT = 12000
LOG = logging.getLogger("lidar_scan_watchdog")


class ScanHealth:
    """Pure state machine; all times are monotonic seconds."""

    def __init__(self, started: float):
        self.started = started
        self.last_scan: float | None = None
        self.last_outage_report: float | None = None
        self.outage_started: float | None = None
        self.baseline_saved = False

    def scan(self, now: float) -> list[tuple[str, float | None]]:
        events: list[tuple[str, float | None]] = []
        if self.outage_started is not None:
            events.append(("recovery", now - self.outage_started))
            self.outage_started = None
            self.last_outage_report = None
        self.last_scan = now
        if not self.baseline_saved:
            self.baseline_saved = True
            events.append(("baseline", None))
        return events

    def check(self, now: float) -> list[tuple[str, float | None]]:
        if now - self.started < STARTUP_GRACE:
            return []
        last = self.last_scan if self.last_scan is not None else self.started
        if now - last < SCAN_TIMEOUT:
            return []
        if self.outage_started is None:
            self.outage_started = now
            self.last_outage_report = now
            return [("outage", now - last)]
        if self.last_outage_report is not None and now - self.last_outage_report >= REPEAT_INTERVAL:
            self.last_outage_report = now
            return [("outage_repeat", now - last)]
        return []


def command(argv: list[str], timeout: int = 5) -> dict[str, object]:
    try:
        result = subprocess.run(argv, capture_output=True, text=True, errors="replace", timeout=timeout, check=False)
        return {"exit_code": result.returncode, "output": (result.stdout + result.stderr)[:MAX_OUTPUT]}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"error": str(error)[:500]}


def snapshot() -> dict[str, object]:
    runtime = Path(os.environ.get("XDG_RUNTIME_DIR", "/run/user/%d" % os.getuid())) / "mobile-robot-devices"
    serial_ports = sorted(glob.glob("/dev/ttyUSB*"))
    links = {name: os.path.realpath(runtime / name) if (runtime / name).exists() else None for name in ("lidar", "esp32")}
    usb_power = {}
    for path in glob.glob("/sys/bus/usb/devices/*/power/control"):
        try:
            usb_power[path] = Path(path).read_text().strip()
        except OSError as error:
            usb_power[path] = str(error)
    checks = {
        "robot_services": ["systemctl", "--user", "show", "rplidar.service", "esp32-serial-bridge.service", "device-core-discovery.service", "rgb-object-detection.service", "gamepad-joy.service", "robot-teleop.service", "tof-depth-publisher.service", "soarm-bridge.service", "-p", "Id", "-p", "ActiveState", "-p", "SubState", "-p", "MainPID", "-p", "NRestarts"],
        "rgb_enabled": ["systemctl", "--user", "is-enabled", "rgb-object-detection.service"],
        "ros_topics": ["ros2", "topic", "list", "-t"],
        "ros_nodes": ["ros2", "node", "list"],
        "usb_tree": ["lsusb", "-t"],
        "usb_devices": ["lsusb"],
        "serial_owners": ["fuser", "-v", *serial_ports] if serial_ports else ["fuser", "-v", "/dev/ttyUSB0"],
        "temperature": ["vcgencmd", "measure_temp"],
        "throttling": ["vcgencmd", "get_throttled"],
        "processes": ["ps", "-eo", "pid,ppid,stat,%cpu,%mem,etime,args", "--sort=-%cpu"],
        "lidar_log": ["journalctl", "--user", "-b", "-u", "rplidar.service", "--since=-3min", "--no-pager", "-n", "100"],
        "robot_log": ["journalctl", "--user", "-b", "-u", "esp32-serial-bridge.service", "-u", "rgb-object-detection.service", "-u", "tof-depth-publisher.service", "--since=-3min", "--no-pager", "-n", "100"],
        "kernel_log": ["journalctl", "-k", "-b", "--since=-3min", "--no-pager", "-n", "100"],
    }
    return {"serial_ports": serial_ports, "runtime_links": links, "usb_power_control": usb_power,
            "checks": {name: command(argv) for name, argv in checks.items()}}


def write_event(directory: Path, event: str, value: float | None) -> Path:
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    stamp = datetime.now(timezone.utc)
    record: dict[str, object] = {"event": event, "timestamp_utc": stamp.isoformat(), "seconds": value}
    if event != "recovery":
        record["snapshot"] = snapshot()
    path = directory / f"{stamp:%Y%m%dT%H%M%S}-{event}-{uuid.uuid4().hex[:8]}.json"
    temporary = path.with_suffix(".tmp")
    try:
        fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(record, stream, indent=2)
            stream.write("\n")
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)
    files = sorted(directory.glob("*.json"), key=lambda file: file.stat().st_mtime)
    for old in files[:-MAX_SNAPSHOTS]:
        old.unlink()
    return path


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    directory = Path.home() / ".local/state/mobile-robot/lidar-watchdog"
    events: queue.Queue[tuple[str, float | None]] = queue.Queue(maxsize=8)
    health = ScanHealth(time.monotonic())

    def reporter() -> None:
        while True:
            event, value = events.get()
            try:
                path = write_event(directory, event, value)
                LOG.info("%s seconds=%s evidence=%s", event, value, path)
            except Exception:
                LOG.exception("Could not save %s evidence", event)
            finally:
                events.task_done()

    def enqueue(items: list[tuple[str, float | None]]) -> None:
        for item in items:
            try:
                events.put_nowait(item)
            except queue.Full:
                LOG.error("Evidence queue full; dropped %s", item[0])

    threading.Thread(target=reporter, name="evidence-writer", daemon=True).start()
    rclpy.init()
    node = Node("lidar_scan_watchdog")
    node.create_subscription(LaserScan, "/scan", lambda _message: enqueue(health.scan(time.monotonic())), qos_profile_sensor_data)
    node.create_timer(1.0, lambda: enqueue(health.check(time.monotonic())))
    LOG.info("Monitoring /scan; timeout=%ss startup_grace=%ss; evidence=%s", SCAN_TIMEOUT, STARTUP_GRACE, directory)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
