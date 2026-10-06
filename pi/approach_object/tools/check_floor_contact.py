#!/usr/bin/env python3
"""Stationary floor-contact diagnostic; no nearest-surface clearance or motion.

Requires explicit native ToF floor mask and RGB object mask. Inspect every
frame for stationary object/cameras and a visible, unobstructed floor contact.
Nominal geometry is for exploring feasibility, never physical qualification.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
import yaml
from approach_object.floor_range import fit_plane, intersect_ray
from fov_overlap_preview import nominal_k


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture', required=True)
    p.add_argument('--rgb-mask', required=True)
    p.add_argument('--floor-mask', required=True)
    p.add_argument('--confidence-min', required=True, type=float)
    geometry = p.add_mutually_exclusive_group(required=True)
    geometry.add_argument('--calibration')
    geometry.add_argument('--nominal', action='store_true')
    p.add_argument('--rgb-diagonal-deg', type=float, default=90)
    p.add_argument('--tof-diagonal-deg', type=float, default=70)
    p.add_argument('--baseline-x-m', type=float, default=.04)
    a = p.parse_args()
    if (not np.isfinite(a.confidence_min) or a.confidence_min < 0
            or not all(0 < v < 180 for v in (a.rgb_diagonal_deg, a.tof_diagonal_deg))
            or not np.isfinite(a.baseline_x_m)):
        p.error('invalid quality or geometry parameters')
    root = Path(a.capture)
    folders = sorted(root.glob('frame-*')) or [root]
    object_mask = cv2.imread(a.rgb_mask, cv2.IMREAD_GRAYSCALE)
    floor_mask = cv2.imread(a.floor_mask, cv2.IMREAD_GRAYSCALE)
    if object_mask is None or floor_mask is None:
        p.error('mask unreadable')
    rgb = cv2.imread(str(folders[0]/'rgb.png'))
    if rgb is None or object_mask.shape != rgb.shape[:2]:
        p.error('RGB object mask dimensions differ from capture')
    v, u = np.where(object_mask > 0)
    if len(v) < 12 or v.max() >= object_mask.shape[0]-2:
        p.error('object absent or bottom clipped; visible floor contact required')
    contact = np.array([np.median(u[v >= v.max()-2]), v.max()], dtype=float)
    if a.nominal:
        t = np.eye(4)
        t[0, 3] = a.baseline_x_m
        c = dict(rgb_K=nominal_k(object_mask.shape, a.rgb_diagonal_deg),
                 tof_K=nominal_k(floor_mask.shape, a.tof_diagonal_deg),
                 rgb_D=[0]*5, tof_D=[0]*5, tof_to_rgb=t.tolist(),
                 depth_semantics='radial')
    else:
        c = yaml.safe_load(Path(a.calibration).read_text())
        if not all(c.get(k) is True for k in ('intrinsics_measured', 'alignment_measured', 'depth_validated')):
            p.error('measured alignment/intrinsics and validated depth required')
    y, x = np.indices(floor_mask.shape)
    uv = np.column_stack((x.ravel(), y.ravel())).astype(float)
    rays = cv2.undistortPoints(uv.reshape(-1, 1, 2), np.array(c['tof_K'], float), np.array(c['tof_D'], float)).reshape(-1, 2)
    rays = np.column_stack((rays, np.ones(len(rays))))
    if c['depth_semantics'] == 'radial':
        rays /= np.linalg.norm(rays, axis=1)[:, None]
    elif c['depth_semantics'] != 'optical_z':
        p.error('invalid depth interpretation')
    contact_ray = cv2.undistortPoints(contact.reshape(1, 1, 2), np.array(c['rgb_K'], float), np.array(c['rgb_D'], float)).ravel()
    inverse = np.linalg.inv(np.array(c['tof_to_rgb'], float))
    origin = inverse[:3, 3]
    direction = inverse[:3, :3] @ np.r_[contact_ray, 1]
    frames = []
    for f in folders:
        entry = dict(frame=f.name, contact_range_m=None)
        try:
            depth = np.load(f/'depth-mm.npy', allow_pickle=False)*.001
            confidence = np.load(f/'confidence.npy', allow_pickle=False)
            if depth.shape != floor_mask.shape or confidence.shape != depth.shape:
                raise ValueError('ToF dimensions changed')
            selected = ((floor_mask.ravel() > 0) & np.isfinite(depth.ravel())
                        & (depth.ravel() >= .05) & (depth.ravel() <= 4)
                        & np.isfinite(confidence.ravel())
                        & (confidence.ravel() >= a.confidence_min))
            points = rays[selected]*depth.ravel()[selected, None]
            normal, offset, quality = fit_plane(points)
            hit = intersect_ray(normal, offset, origin, direction)
            entry.update(contact_range_m=float(np.linalg.norm(hit-origin)),
                         contact_in_tof_m=hit.tolist(), plane_quality=quality)
        except ValueError as error:
            entry['reason'] = str(error)
        frames.append(entry)
    values = [f['contact_range_m'] for f in frames if f['contact_range_m'] is not None]
    report = dict(mode='NOMINAL_FLOOR_CONTACT_DIAGNOSTIC' if a.nominal else 'STATIONARY_FLOOR_CONTACT_DIAGNOSTIC',
                  motion_authority=False, front_clearance_m=None,
                  rgb_contact_uv=contact.tolist(), frames=frames,
                  successful_frames=len(values), total_frames=len(frames),
                  rgb_mask_source=a.rgb_mask, floor_mask_source=a.floor_mask,
                  confidence_min=a.confidence_min,
                  assumptions=['stationary cameras and object throughout burst',
                               'supplied ToF region is floor', 'RGB mask bottom is floor contact'],
                  reason='contact-point range is not nearest-surface front clearance; physical accuracy unverified')
    if a.nominal:
        report.update(rgb_diagonal_deg=a.rgb_diagonal_deg, tof_diagonal_deg=a.tof_diagonal_deg,
                      baseline_x_m=a.baseline_x_m, depth_semantics='assumed radial',
                      geometry_assumptions=['parallel optical axes', 'centred uncropped pinhole images'])
    if values:
        report['contact_range_percentiles_m'] = np.percentile(values, [5, 50, 95]).tolist()
    (root/'floor-contact-quality.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k != 'frames'}, indent=2))


if __name__ == '__main__':
    main()
