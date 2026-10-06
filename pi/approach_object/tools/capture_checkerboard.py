#!/usr/bin/env python3
"""Capture and check a printed OpenCV 9x6 inner-corner board in RGB and ToF IR.

Stores observations only; does not qualify calibration or command motion.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from capture_preview import data_file
from capture_current_rgb_tof import main as capture_main


def detect(gray):
    scale = 3 if gray.shape[1] < 400 else 1
    work = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    ok, corners = cv2.findChessboardCornersSB(
        work, (9, 6), cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
    if not ok:
        ok, corners = cv2.findChessboardCorners(
            work, (9, 6), cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ok:
            corners = cv2.cornerSubPix(work, corners, (5, 5), (-1, -1),
                                      (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, .001))
    if not ok:
        return None
    # Invert OpenCV resize's pixel-centre transform to native coordinates.
    return (corners + .5) / scale - .5


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, help='Check an existing capture instead of acquiring one')
    parser.add_argument('--output', type=Path, default=Path('captures/calibration'))
    parser.add_argument('--square-mm', type=float, help='Measured printed square width; never inferred from paper size')
    args = parser.parse_args()
    if args.square_mm is not None and (not np.isfinite(args.square_mm) or args.square_mm <= 0):
        parser.error('--square-mm must be finite and positive')
    if args.capture:
        folder = args.capture.resolve()
    else:
        args.output.mkdir(parents=True, exist_ok=True)
        index = 1
        while (args.output / f'view-{index:03d}').exists():
            index += 1
        folder = (args.output / f'view-{index:03d}').resolve()
        import sys
        original = sys.argv
        try:
            sys.argv = [original[0], '--output', str(folder)]
            capture_main()
        finally:
            sys.argv = original
    rgb = cv2.imread(str(folder / 'rgb.png'))
    if rgb is None:
        raise ValueError('RGB capture missing')
    amp = np.load(data_file(folder, 'amplitude.npy'), allow_pickle=False)
    values = np.log1p(np.maximum(np.nan_to_num(amp), 0))
    finite = values[np.isfinite(values)]
    lo, hi = np.percentile(finite, [1, 99])
    ir = (np.clip((values-lo)/max(hi-lo, 1e-6), 0, 1)*255).astype(np.uint8)
    linear = np.maximum(np.nan_to_num(amp, nan=0, posinf=0, neginf=0), 0)
    linear_lo, linear_hi = np.percentile(linear, [1, 99])
    ir_linear = (np.clip((linear-linear_lo)/max(linear_hi-linear_lo, 1e-6), 0, 1)*255).astype(np.uint8)
    observations = {}
    previews = []
    for name, gray, color in [('rgb', cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY), rgb),
                               ('tof', ir, cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR))]:
        corners = detect(gray)
        if corners is None and name == 'tof':
            corners = detect(ir_linear)
            if corners is not None:
                color = cv2.cvtColor(ir_linear, cv2.COLOR_GRAY2BGR)
        found = corners is not None
        observations[name] = dict(found=found, shape=list(gray.shape),
                                  corners_native_xy=None if not found else corners.reshape(-1, 2).tolist())
        view = color.copy()
        if found:
            cv2.drawChessboardCorners(view, (9, 6), corners, True)
        view = cv2.resize(view, (640, 480), interpolation=cv2.INTER_NEAREST if name == 'tof' else cv2.INTER_AREA)
        cv2.rectangle(view, (0, 0), (640, 32), (0, 0, 0), -1)
        cv2.putText(view, name.upper() + (': 54 corners detected' if found else ': board NOT detected'),
                    (8, 23), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 0) if found else (0, 180, 255), 1)
        previews.append(view)
    if not cv2.imwrite(str(folder / 'checkerboard.png'), np.hstack(previews)):
        raise RuntimeError('could not save checkerboard preview')
    metadata = json.loads(data_file(folder, 'capture.json').read_text())
    result = dict(pattern_inner_corners=[9, 6], square_mm=args.square_mm,
                  both_detected=all(v['found'] for v in observations.values()),
                  observations=observations, timestamp_delta_s=metadata['delta_s'],
                  qualified=False, motion_commanded=False,
                  note='Corner ordering between modalities must be verified before stereo fitting.')
    data = folder / 'data'
    data.mkdir(exist_ok=True)
    (data / 'checkerboard.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(capture=str(folder), both_detected=result['both_detected'],
                         rgb_detected=observations['rgb']['found'],
                         tof_detected=observations['tof']['found'], square_mm=args.square_mm)))


if __name__ == '__main__':
    main()
