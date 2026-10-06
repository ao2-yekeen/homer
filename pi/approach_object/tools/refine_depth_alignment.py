#!/usr/bin/env python3
"""Refine RGB extrinsics against recorded ToF checkerboard surfaces.

Unlike image-only stereo fitting, these correspondences use the depth values
that the runtime will project. No distance offset or object-specific fit is used.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[3]


def rays(uv, config):
    xy = cv2.undistortPoints(np.asarray(uv, float).reshape(-1, 1, 2),
                            np.asarray(config['tof_K'], float),
                            np.asarray(config['tof_D'], float)).reshape(-1, 2)
    return np.column_stack((xy, np.ones(len(xy))))


def measured_corners(folder, config):
    observation = json.loads((folder/'data/checkerboard.json').read_text())
    if not observation['both_detected']:
        raise ValueError(f'{folder}: both checkerboards must be detected')
    rgb = np.asarray(observation['observations']['rgb']['corners_native_xy'], float)
    tof = np.asarray(observation['observations']['tof']['corners_native_xy'], float)
    if rgb.shape != (54, 2) or tof.shape != (54, 2):
        raise ValueError('expected 9 by 6 checkerboard corners')
    for key in ('rgb', 'tof'):
        if observation['observations'][key]['shape'] != config[key+'_shape']:
            raise ValueError('checkerboard resolution differs from calibration')
    a, b = rgb[[8, 45]]-rgb[0], tof[[8, 45]]-tof[0]
    cos = np.sum(a*b, axis=1)/(np.linalg.norm(a, axis=1)*np.linalg.norm(b, axis=1))
    if cos.mean() < 0:
        tof = tof[::-1].copy(); cos = -cos
    if cos.min() < .5:
        raise ValueError('inconsistent checkerboard orientation')
    depth = np.load(folder/'data/depth-mm.npy', allow_pickle=False)*.001
    mask = np.zeros(depth.shape, np.uint8)
    cv2.fillConvexPoly(mask, cv2.convexHull(tof.astype(np.float32)).astype(int), 1)
    mask = cv2.erode(mask, np.ones((5, 5), np.uint8)) > 0
    valid = mask & np.isfinite(depth) & (depth > config['depth_min_m']) & (depth < config['depth_max_m'])
    if config.get('confidence_min') is not None:
        confidence = np.load(folder/'data/confidence.npy', allow_pickle=False)
        valid &= np.isfinite(confidence) & (confidence >= config['confidence_min'])
    y, x = np.where(valid)
    direction = rays(np.column_stack((x, y)), config)
    if config['depth_semantics'] == 'radial':
        direction /= np.linalg.norm(direction, axis=1)[:, None]
    elif config['depth_semantics'] != 'optical_z':
        raise ValueError('unknown depth semantics')
    xyz = direction*depth[y, x, None]
    if len(xyz) < 100:
        raise ValueError('too few reliable board depth samples')
    use = np.ones(len(xyz), bool)
    for _ in range(8):
        if use.sum() < 100:
            raise ValueError('too few board plane inliers')
        centre = np.mean(xyz[use], axis=0)
        _, _, vt = np.linalg.svd(xyz[use]-centre, full_matrices=False)
        normal = vt[-1]
        residual = (xyz-centre)@normal
        mad = np.median(np.abs(residual[use]-np.median(residual[use])))
        use = np.abs(residual) <= max(.008, 3*1.4826*mad)
    rms = float(np.sqrt(np.mean(residual[use]**2)))
    if rms > .02 or use.mean() < .7:
        raise ValueError('board depth is not a well supported plane')
    corner_rays = rays(tof, config)
    denom = corner_rays@normal
    if np.min(np.abs(denom)) < .1:
        raise ValueError('board nearly edge-on')
    corners = corner_rays*((centre@normal)/denom)[:, None]
    if np.any(corners[:, 2] <= 0):
        raise ValueError('board behind ToF camera')
    return dict(folder=folder, xyz=corners, rgb=rgb, plane_rms_mm=rms*1000,
                samples=int(use.sum()), inlier_fraction=float(use.mean()))


def solve(views, config):
    xyz = np.concatenate([v['xyz'] for v in views])
    uv = np.concatenate([v['rgb'] for v in views])
    k, d = np.asarray(config['rgb_K'], float), np.asarray(config['rgb_D'], float)
    ok, rv, tv = cv2.solvePnP(xyz, uv, k, d, flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        raise ValueError('depth-to-RGB pose solve failed')
    rv, tv = cv2.solvePnPRefineLM(xyz, uv, k, d, rv, tv)
    matrix = np.eye(4)
    matrix[:3, :3] = cv2.Rodrigues(rv)[0]; matrix[:3, 3] = tv.ravel()
    if np.linalg.norm(tv) > .25:
        raise ValueError('fitted camera separation exceeds 25 cm')
    return matrix


def errors(view, matrix, config):
    xyz = view['xyz']@matrix[:3, :3].T+matrix[:3, 3]
    if np.any(xyz[:, 2] <= 0):
        raise ValueError('board behind RGB camera')
    uv = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3), np.asarray(config['rgb_K'], float),
                           np.asarray(config['rgb_D'], float))[0].reshape(-1, 2)
    return uv, np.linalg.norm(uv-view['rgb'], axis=1)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--views', nargs='+', required=True, type=Path)
    p.add_argument('--calibration', type=Path, default=REPO/'pi/approach_object/config/calibration.yaml')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    config = yaml.safe_load(args.calibration.read_text())
    folders = [v.resolve() for v in args.views]
    if len(folders) < 3 or len(set(folders)) != len(folders):
        p.error('at least three distinct captures are required')
    hashes = [hashlib.sha256((v/'rgb.png').read_bytes()).hexdigest() for v in folders]
    if len(set(hashes)) != len(hashes):
        p.error('duplicate images cannot be held-out views')
    views = [measured_corners(v, config) for v in folders]
    checks = []
    for i, view in enumerate(views):
        matrix = solve(views[:i]+views[i+1:], config)
        _, e = errors(view, matrix, config)
        checks.append(dict(view=str(view['folder']), rms_rgb_px=float(np.sqrt(np.mean(e**2))),
                           p95_rgb_px=float(np.percentile(e, 95))))
    if max(c['rms_rgb_px'] for c in checks) > 4 or max(c['p95_rgb_px'] for c in checks) > 8:
        raise ValueError('held-out depth projection failed; calibration not saved')
    matrix = solve(views, config)
    before, after, tiles = [], [], []
    for view in views:
        _, old = errors(view, np.asarray(config['tof_to_rgb']), config)
        uv, new = errors(view, matrix, config)
        before.extend(old); after.extend(new)
        im = cv2.imread(str(view['folder']/'rgb.png'))
        for actual, projected in zip(view['rgb'], uv):
            cv2.circle(im, tuple(np.rint(actual).astype(int)), 3, (0,255,0), 1)
            cv2.circle(im, tuple(np.rint(projected).astype(int)), 2, (255,0,255), -1)
        tiles.append(cv2.resize(im, (480,360)))
    before_rms, after_rms = [float(np.sqrt(np.mean(np.square(e)))) for e in (before, after)]
    if after_rms >= before_rms:
        raise ValueError('depth projection did not improve; calibration not saved')
    out = args.output.resolve()
    if out.exists() and any(out.iterdir()):
        p.error('output must be fresh or empty')
    out.mkdir(parents=True, exist_ok=True)
    (out/'previous-calibration.yaml').write_text(args.calibration.read_text())
    config.update(tof_to_rgb=matrix.tolist(), alignment_source='depth-anchored checkerboard planes; provisional held-out projection checks',
                  qualified=False, enable_motion=False, alignment_measured=False)
    (out/'calibration.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    report = dict(before_depth_projection_rms_px=before_rms, after_depth_projection_rms_px=after_rms,
                  held_out_checks=checks, tof_to_rgb=matrix.tolist(), qualified=False,
                  boards=[{k: str(v) if isinstance(v, Path) else v for k,v in b.items() if k not in ('xyz','rgb')} for b in views],
                  limitations=['Uses recorded depth without correcting sensor range bias.',
                               'Intrinsic parameters and depth semantics remain provisional.',
                               'Three views do not qualify all distances or camera poses.'])
    (out/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    while len(tiles)%2: tiles.append(np.zeros_like(tiles[0]))
    cv2.imwrite(str(out/'alignment.png'), np.vstack([np.hstack(tiles[i:i+2]) for i in range(0,len(tiles),2)]))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
