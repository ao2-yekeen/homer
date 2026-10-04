#!/usr/bin/env python3
"""Offline recognition screening. No ROS, camera access or actuator commands.

Use an isolated workstation environment with ultralytics==8.4.173.
Confidence scores and these few images do not establish reliability or Pi latency.
"""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import time

import numpy as np
from ultralytics import YOLO, YOLOE
from ultralytics.models.yolo.yoloe.predict import YOLOEVPSegPredictor


def fingerprint(path):
    path = Path(path)
    return dict(name=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', action='append', required=True)
    p.add_argument('--image', action='append', required=True)
    p.add_argument('--sizes', nargs='+', type=int, default=[320, 640])
    p.add_argument('--reference', help='Optional example image for visual instance matching')
    p.add_argument('--box', nargs=4, type=float, metavar=('X1', 'Y1', 'X2', 'Y2'))
    p.add_argument('--output', required=True)
    a = p.parse_args()
    if bool(a.reference) != bool(a.box):
        p.error('--reference and --box must be supplied together')
    if a.box and (a.box[0] >= a.box[2] or a.box[1] >= a.box[3]):
        p.error('reference box must have positive width and height')
    result = dict(
        qualification='SCREENING_ONLY; no Pi timing, class reliability or motion qualification',
        platform=platform.platform(),
        dependencies={n: importlib.metadata.version(n) for n in ('ultralytics', 'torch', 'numpy')},
        diagnostic_confidence=0.05, images=[fingerprint(i) for i in a.image], rows=[],
    )
    if a.reference:
        result['reference'] = dict(**fingerprint(a.reference), box=a.box,
                                   meaning='user-confirmed instance example; not autonomous class recognition')
    for path in a.model:
        if not Path(path).is_file():
            p.error('download and review local checkpoint first: ' + path)
        model = (YOLOE if a.reference else YOLO)(path, task='segment')
        for image in a.image:
            for size in a.sizes:
                kwargs = {}
                if a.reference:
                    kwargs = dict(predictor=YOLOEVPSegPredictor, refer_image=a.reference,
                                  visual_prompts=dict(bboxes=np.array([a.box]), cls=np.array([0])))
                start = time.monotonic()
                prediction = model.predict(image, imgsz=size, conf=0.05, verbose=False,
                                           device='cpu', **kwargs)[0]
                row = dict(model=fingerprint(path), image=Path(image).name, imgsz=size,
                           wall_s=time.monotonic()-start,
                           detections=[dict(label=prediction.names[int(b.cls[0])],
                                            score=float(b.conf[0]), box=b.xyxy[0].tolist())
                                       for b in prediction.boxes])
                result['rows'].append(row)
                Path(a.output).write_text(json.dumps(result, indent=2)+'\n')
                print(json.dumps(row), flush=True)


if __name__ == '__main__':
    main()
