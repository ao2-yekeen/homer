#!/usr/bin/env python3
"""Capture an RGB/ToF pair, preserving each capture in its own folder."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import shutil
import sys
import uuid

import cv2
import numpy as np
from capture_preview import depth_preview

PACKAGE = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE / "tools"))
from run_named_object_distance_test import capture_pi


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pi", default="pi", help="SSH host alias (default: pi)")
    parser.add_argument("--output", type=Path,
                        help="New capture folder; default: captures/current/<timestamp>")
    args = parser.parse_args()

    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    destination = args.output or REPO_ROOT / 'captures' / 'current' / (stamp + '-' + uuid.uuid4().hex[:6])
    capture = destination.expanduser().resolve()
    if capture.exists():
        parser.error(f"capture folder already exists: {capture}; choose a new --output folder")
    capture_pi(args.pi, capture, 1)

    depth = np.load(capture / "depth-mm.npy", allow_pickle=False)
    amplitude_path = capture / 'amplitude.npy'
    amplitude = np.load(amplitude_path, allow_pickle=False) if amplitude_path.is_file() else None
    tof_image = depth_preview(depth, amplitude=amplitude)
    if not cv2.imwrite(str(capture / "tof.png"), tof_image):
        raise RuntimeError("could not save ToF preview")

    data = capture / 'data'
    data.mkdir()
    for name in ('depth-mm.npy', 'amplitude.npy', 'confidence.npy', 'capture.json',
                 'amplitude.png', 'confidence.png'):
        source = capture / name
        if source.is_file():
            shutil.move(str(source), data / name)

    print(f"RGB image: {capture / 'rgb.png'}")
    print(f"Extra USB camera image: {capture / 'usb-camera.png'}")
    print(f"ToF image: {capture / 'tof.png'}")


if __name__ == "__main__":
    main()
