#!/usr/bin/env python3
"""Nominal field-of-view overlap from datasheets and mounting geometry. No matching/motion.

Pinhole lens, centred uncropped images and parallel optical axes are assumptions.
This preview never supplies a robot clearance or qualifies autonomous movement.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from approach_object.geometry import project_depth


def nominal_k(shape, diagonal_deg):
    h, w = shape
    f = np.hypot(w, h) / (2*np.tan(np.deg2rad(diagonal_deg)/2))
    return [[f, 0, (w-1)/2], [0, f, (h-1)/2], [0, 0, 1]]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture', required=True)
    p.add_argument('--rgb-diagonal-deg', type=float, required=True)
    p.add_argument('--tof-diagonal-deg', type=float, required=True)
    p.add_argument('--baseline-x-m', type=float, required=True,
                   help='ToF centre position in RGB optical frame: x right, metres')
    a = p.parse_args()
    if not all(0 < v < 180 for v in (a.rgb_diagonal_deg, a.tof_diagonal_deg)) or not np.isfinite(a.baseline_x_m):
        p.error('invalid nominal geometry')
    root = Path(a.capture)
    rgb = cv2.imread(str(root / 'rgb.png'))
    if rgb is None:
        p.error('RGB image unreadable')
    depth = np.load(root / 'depth-mm.npy', allow_pickle=False)
    m = np.eye(4)
    m[0, 3] = a.baseline_x_m
    c = dict(rgb_shape=rgb.shape[:2], tof_shape=depth.shape,
             rgb_K=nominal_k(rgb.shape[:2], a.rgb_diagonal_deg), rgb_D=[0]*5,
             tof_K=nominal_k(depth.shape, a.tof_diagonal_deg), tof_D=[0]*5,
             tof_to_rgb=m.tolist(), depth_semantics='radial',
             depth_min_m=.05, depth_max_m=4., min_samples=12)
    # Project nominal ToF frustum corners at several optical depths. Offset varies
    # with distance because the sensors have different centres (parallax).
    h, w = depth.shape
    k = np.array(c['tof_K'])
    rays = np.array([[(u-k[0,2])/k[0,0], (v-k[1,2])/k[1,1], 1]
                     for u, v in [(0,0), (w-1,0), (w-1,h-1), (0,h-1)]])
    footprints = []
    for z, colour in [(.5, (0,255,255)), (1., (255,255,0)), (2., (255,0,255))]:
        points = rays*z + m[:3,3]
        pixels, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), np.array(c['rgb_K']), np.zeros(5))
        poly = np.rint(pixels.reshape(-1,2)).astype(np.int32)
        cv2.polylines(rgb, [poly], True, colour, 2)
        cv2.putText(rgb, f'nominal ToF FOV at {z} m', tuple(poly[0]), cv2.FONT_HERSHEY_SIMPLEX, .4, colour, 1)
        footprints.append(dict(optical_plane_m=z, rgb_corners=poly.tolist()))
    report = dict(mode='NOMINAL_FOV_PREVIEW', motion_authority=False, clearance_m=None,
                  assumptions=['parallel optical axes', 'centred uncropped pinhole images',
                               'ToF radial millimetres', 'datasheet FOV applies to current modes'],
                  rgb_diagonal_deg=a.rgb_diagonal_deg, tof_diagonal_deg=a.tof_diagonal_deg,
                  baseline_x_m=a.baseline_x_m, footprints=footprints,
                  reason='approximate overlap geometry; depth quality and front-edge transform unvalidated')
    mask_path = root / 'bottle-mask.png'
    if mask_path.exists():
        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        pix, points = project_depth(depth, c)
        selected = cv2.erode(mask, np.ones((3,3), np.uint8))[pix[:,1], pix[:,0]] > 0
        values = np.linalg.norm(points[selected], axis=1)
        report['nominal_mask_depth_samples'] = int(len(values))
        if len(values):
            report['raw_selected_range_percentiles_m'] = np.percentile(values, [5,50,95]).tolist()
            report['raw_selected_span_m'] = float(np.ptp(values))
            report['reason'] += '; selected sample statistics are not a verified bottle distance'
        rgb[mask > 0] = (.7*rgb[mask > 0] + .3*np.array([0,255,0])).astype(np.uint8)
    cv2.putText(rgb, 'APPROXIMATE FOV - NO VERIFIED DISTANCE', (5,20), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255), 1)
    cv2.imwrite(str(root / 'fov-overlap.png'), rgb)
    (root / 'fov-overlap.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
