#!/usr/bin/env python3
"""Fit RGB/ToF stereo geometry from saved paired checkerboards, with held-out errors."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[3]


def fit(views, obj, model):
    rgb_shape, tof_shape = views[0]['shapes']
    flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3
    if model == 'centred_pinhole':
        flags |= cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2
    solutions = []
    for focal in (400., 550.):
        k1 = np.array([[focal, 0, rgb_shape[1]/2], [0, focal, rgb_shape[0]/2], [0, 0, 1.]])
        k2 = np.array([[190.92, 0, tof_shape[1]/2], [0, 191.25, tof_shape[0]/2], [0, 0, 1.]])
        result = cv2.stereoCalibrate(
            [obj]*len(views), [v['rgb'] for v in views], [v['tof'] for v in views],
            k1, np.zeros(5), k2, np.zeros(5), tuple(rgb_shape[::-1]), flags=flags,
            criteria=(cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 150, 1e-8))
        solutions.append(result)
    result = min(solutions, key=lambda x: x[0])
    rms, k1, d1, k2, d2, r, t = result[:7]
    return dict(rms=float(rms), rgb_K=k1, rgb_D=d1, tof_K=k2, tof_D=d2, R=r, T=t)


def transferred(view, obj, fitted):
    # Solve the unseen board pose using only its IR image, then predict RGB.
    ok, rv, tv = cv2.solvePnP(obj, view['tof'], fitted['tof_K'], fitted['tof_D'])
    if not ok:
        raise ValueError('board pose solve failed')
    board_r, _ = cv2.Rodrigues(rv)
    r = fitted['R'].T @ board_r
    t = fitted['R'].T @ (tv-fitted['T'])
    rv, _ = cv2.Rodrigues(r)
    pixels, _ = cv2.projectPoints(obj, rv, t, fitted['rgb_K'], fitted['rgb_D'])
    errors = np.linalg.norm((pixels-view['rgb']).reshape(-1, 2), axis=1)
    return pixels.reshape(-1, 2), errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=REPO/'captures'/'alignment')
    parser.add_argument('--captures', type=Path, help='Use only captures from this mounting session')
    parser.add_argument('--views', nargs='+', type=Path, help='Use exactly these capture folders')
    args = parser.parse_args()
    if args.views is not None:
        files = [view.expanduser().resolve() / 'data/checkerboard.json' for view in args.views]
    elif args.captures is not None:
        files = sorted(args.captures.expanduser().resolve().glob('*/data/checkerboard.json'))
    else:
        files = sorted((REPO/'captures'/'current').glob('*/data/checkerboard.json'))
        files += sorted((REPO/'captures'/'calibration').glob('*/data/checkerboard.json'))
    views, seen, skipped = [], set(), []
    for path in files:
        data = json.loads(path.read_text())
        if not data['both_detected']:
            continue
        if data.get('square_mm') != 23 or data.get('pattern_inner_corners') != [9, 6]:
            skipped.append(str(path)); continue
        folder = path.parent.parent
        digest = hashlib.sha256((folder/'rgb.png').read_bytes()).hexdigest()
        if digest in seen:
            continue
        seen.add(digest)
        rgb = np.array(data['observations']['rgb']['corners_native_xy'], np.float32).reshape(-1, 1, 2)
        tof = np.array(data['observations']['tof']['corners_native_xy'], np.float32).reshape(-1, 1, 2)
        # These cameras share image orientation; resolve the board's 180 degree ambiguity.
        # Reject mismatched orientation instead of silently pairing unrelated corners.
        def orientation(points):
            return np.stack([points[8, 0]-points[0, 0], points[45, 0]-points[0, 0]])
        a, b = orientation(rgb), orientation(tof)
        cos = np.sum(a*b, axis=1)/(np.linalg.norm(a, axis=1)*np.linalg.norm(b, axis=1))
        reverse = bool(np.mean(cos) < 0)
        if reverse:
            tof = tof[::-1].copy(); cos = -cos
        if np.min(cos) < .5:
            skipped.append(str(path)); continue
        views.append(dict(folder=folder, rgb=rgb, tof=tof, reversed=reverse,
                          shapes=[data['observations'][x]['shape'] for x in ('rgb', 'tof')]))
    if len(views) < 3:
        parser.error('at least three paired views are required for this fit and held-out check')
    if any(v['shapes'] != views[0]['shapes'] for v in views):
        parser.error('mixed camera resolutions')
    obj = np.zeros((54, 3), np.float32)
    obj[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)*.023
    candidates = []
    for model in ('centred_pinhole', 'radial_distortion'):
        print(f'Fitting {model} with {len(views)} views and leave-one-out checks...', flush=True)
        checks = []
        for i, view in enumerate(views):
            f = fit(views[:i]+views[i+1:], obj, model)
            _, errors = transferred(view, obj, f)
            checks.append(dict(view=str(view['folder'].relative_to(REPO)),
                               rms_rgb_px=float(np.sqrt(np.mean(errors**2))),
                               p95_rgb_px=float(np.percentile(errors, 95))))
        candidates.append(dict(model=model, checks=checks,
                               mean_held_out_rms_rgb_px=float(np.mean([c['rms_rgb_px'] for c in checks]))))
    chosen = min(candidates, key=lambda c: c['mean_held_out_rms_rgb_px'])
    f = fit(views, obj, chosen['model'])
    matrix = np.eye(4)
    matrix[:3, :3] = f['R'].T
    matrix[:3, 3] = (-f['R'].T @ f['T']).ravel()
    out = args.output.expanduser().resolve(); out.mkdir(parents=True, exist_ok=True)
    tiles = []; depth_errors = {'radial': [], 'optical_z': []}
    for view in views:
        predicted, errors = transferred(view, obj, f)
        im = cv2.imread(str(view['folder']/'rgb.png'))
        for actual, pred in zip(view['rgb'].reshape(-1, 2), predicted):
            a = tuple(np.rint(actual).astype(int)); b = tuple(np.rint(pred).astype(int))
            cv2.line(im, a, b, (0, 0, 255), 1)
            cv2.circle(im, a, 3, (0, 255, 0), 1)
            cv2.circle(im, b, 2, (255, 0, 255), -1)
        cv2.rectangle(im, (0, 0), (im.shape[1], 30), (0, 0, 0), -1)
        cv2.putText(im, f"{view['folder'].name[:22]} RMS {np.sqrt(np.mean(errors**2)):.1f}px",
                    (5, 21), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
        tiles.append(cv2.resize(im, (480, 360)))
        ok, rv, tv = cv2.solvePnP(obj, view['tof'], f['tof_K'], f['tof_D'])
        r, _ = cv2.Rodrigues(rv); points = obj @ r.T + tv.ravel()
        depth = np.load(view['folder']/'data'/'depth-mm.npy')*.001
        xy = np.rint(view['tof'].reshape(-1, 2)).astype(int)
        observed = depth[xy[:, 1], xy[:, 0]]
        valid = np.isfinite(observed) & (observed > .05) & (observed < 2)
        for semantics, prediction in [('radial', np.linalg.norm(points, axis=1)), ('optical_z', points[:, 2])]:
            depth_errors[semantics].extend((observed[valid]-prediction[valid]).tolist())
    while len(tiles)%2:
        tiles.append(np.zeros_like(tiles[0]))
    if not cv2.imwrite(str(out/'alignment.png'), np.vstack([np.hstack(tiles[i:i+2]) for i in range(0,len(tiles),2)])):
        raise RuntimeError('alignment preview write failed')
    depth_check = {key: dict(median_bias_cm=float(np.median(v)*100),
                            median_abs_error_cm=float(np.median(np.abs(v))*100)) for key,v in depth_errors.items()}
    semantics = min(depth_check, key=lambda k: depth_check[k]['median_abs_error_cm'])
    depth_tiles, overlaps = [], []
    for view in views:
        depth = np.load(view['folder']/'data'/'depth-mm.npy')*.001
        native_mask = np.zeros(depth.shape, np.uint8)
        cv2.fillConvexPoly(native_mask, cv2.convexHull(view['tof'].reshape(-1, 2)).astype(np.int32), 1)
        yy, xx = np.where((native_mask > 0) & np.isfinite(depth) & (depth > .05) & (depth < 2))
        uv = np.column_stack([xx, yy]).astype(float)
        rays = cv2.undistortPoints(uv.reshape(-1, 1, 2), f['tof_K'], f['tof_D']).reshape(-1, 2)
        rays = np.column_stack([rays, np.ones(len(rays))])
        if semantics == 'radial':
            rays /= np.linalg.norm(rays, axis=1)[:, None]
        points = rays*depth[yy, xx, None]
        rgb_points = points @ matrix[:3, :3].T + matrix[:3, 3]
        pixels, _ = cv2.projectPoints(rgb_points[rgb_points[:, 2] > 0], np.zeros(3), np.zeros(3), f['rgb_K'], f['rgb_D'])
        im = cv2.imread(str(view['folder']/'rgb.png'))
        actual = cv2.convexHull(view['rgb'].reshape(-1, 2)).astype(np.int32)
        projected = cv2.convexHull(pixels.astype(np.float32)).astype(np.int32)
        actual_mask = np.zeros(im.shape[:2], np.uint8); projected_mask = actual_mask.copy()
        cv2.fillConvexPoly(actual_mask, actual, 1); cv2.fillConvexPoly(projected_mask, projected, 1)
        iou = float(np.count_nonzero(actual_mask & projected_mask)/max(1,np.count_nonzero(actual_mask | projected_mask)))
        overlaps.append(dict(view=str(view['folder'].relative_to(REPO)), board_interior_iou=iou))
        cv2.polylines(im, [actual], True, (0, 255, 0), 2)
        cv2.polylines(im, [projected], True, (255, 0, 255), 2)
        cv2.rectangle(im, (0, 0), (im.shape[1], 30), (0, 0, 0), -1)
        cv2.putText(im, f'Actual depth projection: overlap {iou:.0%}', (5,21), cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
        depth_tiles.append(cv2.resize(im, (480, 360)))
    while len(depth_tiles)%2:
        depth_tiles.append(np.zeros_like(depth_tiles[0]))
    if not cv2.imwrite(str(out/'depth_alignment.png'), np.vstack([np.hstack(depth_tiles[i:i+2]) for i in range(0,len(depth_tiles),2)])):
        raise RuntimeError('depth alignment preview write failed')
    config = yaml.safe_load((REPO/'pi/approach_object/config/calibration.yaml').read_text())
    for name in ('rgb_K', 'tof_K'):
        config[name] = f[name].tolist()
    for name in ('rgb_D', 'tof_D'):
        config[name] = f[name].ravel().tolist()
    config.update(tof_to_rgb=matrix.tolist(), rgb_shape=views[0]['shapes'][0],
                  tof_shape=views[0]['shapes'][1], depth_semantics=semantics,
                  qualified=False, enable_motion=False, intrinsics_measured=False, alignment_measured=False,
                  depth_validated=False, alignment_source='paired checkerboards; provisional fit with held-out errors',
                  depth_semantics_source='provisional choice from board-pose comparison; not independently validated')
    (out/'calibration.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    report = dict(views=len(views), square_mm=23, model=chosen['model'],
                  stereo_training_rms_px=f['rms'], validation=candidates,
                  selected_mean_held_out_rms_rgb_px=chosen['mean_held_out_rms_rgb_px'],
                  depth_check=depth_check, actual_depth_projection=overlaps, provisional_depth_semantics=semantics,
                  tof_to_rgb=matrix.tolist(), skipped=skipped, qualified=False,
                  preview_legend='green: observed RGB corners; magenta: prediction from IR pose; red: residual',
                  limitations=['Existing views are used for leave-one-out checks, not an independent scene.',
                               'Corner orientation assumes similarly oriented camera mounts.',
                               'Mount rigidity across servo tilt has not been verified.'])
    (out/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: report[k] for k in ('views','model','stereo_training_rms_px','selected_mean_held_out_rms_rgb_px','depth_check')}, indent=2))
    print(f'Saved fitted calibration, report and alignment.png in {out}')


if __name__ == '__main__':
    main()
