#!/usr/bin/env python3
"""Capture one current RGB/ToF pair from the Pi and write viewable images."""
from pathlib import Path
import argparse
import shutil
import sys

import cv2
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE / "tools"))
from run_named_object_distance_test import capture_pi


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pi", default="pi", help="SSH host alias (default: pi)")
    parser.add_argument("--output", type=Path, default=REPO_ROOT / "captures" / "current")
    args = parser.parse_args()

    capture = args.output.expanduser().resolve()
    if capture.exists():
        shutil.rmtree(capture)
    capture_pi(args.pi, capture, 1)

    depth = np.load(capture / "depth-mm.npy", allow_pickle=False)
    valid = np.isfinite(depth) & (depth > 0)
    preview = np.zeros(depth.shape, dtype=np.uint8)
    if valid.any():
        low, high = np.percentile(depth[valid], [2, 98])
        scaled = (depth[valid] - low) * (255.0 / max(float(high - low), 1.0))
        preview[valid] = np.clip(scaled, 0, 255).astype(np.uint8)
    tof_image = cv2.applyColorMap(255 - preview, cv2.COLORMAP_TURBO)
    tof_image[~valid] = 0
    if not cv2.imwrite(str(capture / "tof.png"), tof_image):
        raise RuntimeError("could not save ToF preview")

    print(f"RGB image: {capture / 'rgb.png'}")
    print(f"Extra USB camera image: {capture / 'usb-camera.png'}")
    print(f"ToF image: {capture / 'tof.png'}")


if __name__ == "__main__":
    main()
