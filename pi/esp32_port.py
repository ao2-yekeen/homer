"""Find the ESP32 by its HOM1 greeting and maintain its runtime device link."""

from __future__ import annotations

import glob
import os
from pathlib import Path


def candidates(runtime_link: Path) -> list[str]:
    ports = [str(runtime_link), *sorted(glob.glob("/dev/ttyUSB*"))]
    result: list[str] = []
    seen: set[str] = set()
    for port in ports:
        if not os.path.exists(port):
            continue
        actual = os.path.realpath(port)
        if actual not in seen:
            seen.add(actual)
            result.append(port)
    return result


def record_esp32(runtime_link: Path, port: str) -> None:
    runtime_link.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    replacement = runtime_link.with_name(f".{runtime_link.name}.new")
    replacement.unlink(missing_ok=True)
    replacement.symlink_to(os.path.realpath(port))
    replacement.replace(runtime_link)
