#!/usr/bin/env python3
"""Fit a scene-specific preview from manually matched RGB/ToF IR landmarks.

This 2D affine approximation ignores depth-dependent parallax. It neither
qualifies calibration nor changes robot configuration or motion authority.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from capture_preview import data_file, depth_preview


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--rgb-mask', type=Path, required=True)
    parser.add_argument('--landmarks', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    pairs = json.loads(args.landmarks.read_text())
    src = np.asarray([p['rgb_xy'] for p in pairs], dtype=float)
    dst = np.asarray([p['tof_xy'] for p in pairs], dtype=float)
    if len(src) < 4 or src.shape != dst.shape or src.shape[1:] != (2,) or not np.isfinite([src, dst]).all():
        parser.error('provide at least four finite RGB/ToF landmark pairs')
    design = np.c_[src, np.ones(len(src))]
    if np.linalg.matrix_rank(design) < 3:
        parser.error('landmarks must not be collinear')
    matrix = np.linalg.lstsq(design, dst, rcond=None)[0].T
    residuals = np.linalg.norm(design @ matrix.T - dst, axis=1)
    rgb = cv2.imread(str(args.capture / 'rgb.png'))
    mask = cv2.imread(str(args.rgb_mask), cv2.IMREAD_GRAYSCALE)
    depth = np.load(data_file(args.capture, 'depth-mm.npy'), allow_pickle=False)
    amplitude = np.load(data_file(args.capture, 'amplitude.npy'), allow_pickle=False)
    if rgb is None or mask is None or mask.shape != rgb.shape[:2] or amplitude.shape != depth.shape:
        parser.error('capture and mask dimensions do not match')
    h, w = depth.shape
    projected = cv2.warpAffine(mask, matrix, (w, h), flags=cv2.INTER_NEAREST)
    # Exclude the uncertain silhouette boundary from the provisional statistic.
    interior = cv2.erode((projected > 0).astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    accepted = interior & np.isfinite(depth) & (depth > 0) & (depth <= 4000)
    percentiles = np.percentile(depth[accepted] / 10, [5, 50, 95]).tolist() if accepted.any() else None
    values = np.log1p(np.maximum(np.nan_to_num(amplitude), 0))
    lo, hi = np.percentile(values, [1, 99])
    ir = np.clip((values-lo) / max(hi-lo, 1e-6)*255, 0, 255).astype(np.uint8)
    warped = cv2.warpAffine(rgb, matrix, (w, h))
    overlay = cv2.addWeighted(warped, .5, cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR), .5, 0)
    contours, _ = cv2.findContours(projected, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay, contours, -1, (0, 255, 255), 1)
    ir_outline = cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(ir_outline, contours, -1, (0, 255, 255), 1)
    panels = [cv2.resize(p, (720, 540), interpolation=cv2.INTER_NEAREST) for p in (ir_outline, overlay)]
    preview = np.full((600, 1440, 3), 24, np.uint8)
    preview[50:590] = np.concatenate(panels, axis=1)
    cv2.putText(preview, 'ToF infrared + projected RGB mask', (12, 25), 0, .65, (255, 255, 255), 1)
    cv2.putText(preview, 'RGB / infrared overlay | rough scene fit', (735, 25), 0, .65, (255, 255, 255), 1)
    args.output.mkdir(parents=True, exist_ok=True)
    data = args.output / 'data'
    data.mkdir(exist_ok=True)
    cv2.imwrite(str(args.output / 'alignment.png'), preview)
    cv2.imwrite(str(args.output / 'tof-depth.png'), depth_preview(depth, projected))
    cv2.imwrite(str(data / 'projected-mask.png'), projected)
    report = dict(method='manual landmark least-squares affine', qualified=False,
                  motion_authority=False, rgb_to_tof_affine=matrix.tolist(),
                  landmark_pairs=pairs, landmark_errors_tof_px=residuals.tolist(),
                  landmark_rmse_tof_px=float(np.sqrt(np.mean(residuals**2))),
                  provisional_sensor_depth_percentiles_cm=percentiles,
                  accepted_interior_pixels=int(accepted.sum()),
                  rgb_sha256=hashlib.sha256((args.capture/'rgb.png').read_bytes()).hexdigest(),
                  depth_sha256=hashlib.sha256(data_file(args.capture,'depth-mm.npy').read_bytes()).hexdigest(),
                  limitations=['scene-specific; not valid across arbitrary depths or camera movement',
                               'manual feature uncertainty; depth-dependent parallax ignored',
                               'confidence threshold and sensor metric accuracy unqualified',
                               'sensor range is not floor clearance'])
    (args.output / 'result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
