#!/usr/bin/env python3
"""Stationary workstation YOLOE reference mask and calibrated depth overlay. No ROS."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import yaml
from ultralytics import YOLOE
from ultralytics.models.yolo.yoloe.predict import YOLOEVPSegPredictor
from approach_object.geometry import estimate, project_depth

MODEL_SHA256 = '1741c1f8da3cea47e2c01829c334a50dc0b9bbd05e685b90a3ce84fae32c8c1b'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', required=True)
    p.add_argument('--reference', required=True)
    p.add_argument('--box', nargs=4, type=float, required=True)
    p.add_argument('--capture', required=True)
    p.add_argument('--calibration', required=True)
    p.add_argument('--view', type=int, choices=(0, 1), required=True)
    a = p.parse_args()
    if hashlib.sha256(Path(a.model).read_bytes()).hexdigest() != MODEL_SHA256:
        p.error('checkpoint differs from screened YOLOE-26n-seg')
    if a.box[0] >= a.box[2] or a.box[1] >= a.box[3]:
        p.error('reference box must have positive area')
    root = Path(a.capture)
    meta = json.loads((root / 'capture.json').read_text())
    c = yaml.safe_load(Path(a.calibration).read_text())
    image = cv2.imread(str(root / 'rgb.png'))
    if image is None:
        p.error('RGB image unreadable')
    depth = np.load(root / 'depth-mm.npy', allow_pickle=False)
    result = YOLOE(a.model, task='segment').predict(
        image, imgsz=320, conf=.5, device='cpu', verbose=False,
        predictor=YOLOEVPSegPredictor, refer_image=a.reference,
        visual_prompts=dict(bboxes=np.array([a.box]), cls=np.array([0])))[0]
    report = dict(mode='STATIONARY_REFERENCE_MATCHING', motion_authority=False,
                  clearance_m=None, reason='', matches=len(result.boxes),
                  checkpoint_sha256=MODEL_SHA256,
                  reference_sha256=hashlib.sha256(Path(a.reference).read_bytes()).hexdigest(),
                  reference_box=a.box, capture=meta)
    try:
        if len(result.boxes) != 1 or result.masks is None:
            raise ValueError('target absent or multiple reference matches')
        report.update(score=float(result.boxes.conf[0]), bbox=result.boxes.xyxy[0].tolist())
        mask = result.masks.data[0].cpu().numpy()
        mh, mw = mask.shape
        h, w = image.shape[:2]
        gain = min(mw/w, mh/h)
        px, py = (mw-w*gain)/2, (mh-h*gain)/2
        mask = mask[round(py-.1):mh-round(py+.1), round(px-.1):mw-round(px+.1)]
        mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST) > .5
        image[mask] = (.6*image[mask] + .4*np.array([0, 255, 0])).astype(np.uint8)
        cv2.imwrite(str(root / 'bottle-mask.png'), mask.astype(np.uint8)*255)
        if not all(c.get(k) is True for k in ('intrinsics_measured', 'alignment_measured', 'poses_measured', 'depth_validated')):
            raise ValueError('CALIBRATION_BLOCKER: measured intrinsics, alignment, view pose and depth interpretation required')
        if meta['rgb_frame'] != c['rgb_frame'] or meta['tof_frame'] != c['tof_frame']:
            raise ValueError('unexpected capture coordinate frames')
        if abs(meta['rgb_stamp']-meta['depth_stamp']) > .08:
            raise ValueError('unsynchronised capture')
        clearance, bearing = estimate(depth, mask, c, a.view)
        pix, points = project_depth(depth, c)
        distances = np.linalg.norm(points, axis=1)
        for (u, v), distance in zip(pix, distances):
            colour = (255, int(255*min(distance/2, 1)), 0)
            cv2.circle(image, (int(u), int(v)), 1, colour, -1)
        report.update(clearance_m=clearance, bearing_rad=bearing,
                      reason='stationary estimate; tape-measure verification required')
    except (ValueError, TypeError, KeyError) as error:
        report['reason'] = str(error)
    cv2.imwrite(str(root / 'rgb-yoloe-depth.png'), image)
    (root / 'distance.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
