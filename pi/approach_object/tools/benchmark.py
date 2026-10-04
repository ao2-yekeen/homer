#!/usr/bin/env python3
"""Offline Pi inference benchmark; never opens a camera or publishes drive commands."""
import argparse
import json
import platform
import time
import subprocess
from pathlib import Path
import numpy as np
from ultralytics import YOLO
from approach_object.model_io import verify_model

p = argparse.ArgumentParser()
p.add_argument('model')
p.add_argument('--image', required=True, help='Captured RGB image; no direct camera access')
p.add_argument('--samples', type=int, default=30)
p.add_argument('--output', required=True)
a = p.parse_args()
def thermal():
    try:
        return {'temperature_c': int(Path('/sys/class/thermal/thermal_zone0/temp').read_text())/1000, 'throttled': subprocess.check_output(['vcgencmd','get_throttled'],text=True).strip()}
    except (OSError, ValueError, subprocess.CalledProcessError):
        return {}

manifest = verify_model(a.model)
before = thermal()
model = YOLO(a.model, task='segment')
for _ in range(3): model.predict(a.image, imgsz=320, verbose=False)
times = []
for _ in range(a.samples):
    start = time.monotonic()
    model.predict(a.image, imgsz=320, verbose=False)
    times.append(time.monotonic()-start)
result = dict(thermal_before=before, thermal_after=thermal(), platform=platform.platform(), samples=a.samples, input=a.image, inference_p50_s=float(np.quantile(times,.5)), inference_p99_s=float(np.quantile(times,.99)), durations_s=times, model=manifest, qualification='INFERENCE_ONLY; capture buffering and end-to-end latency still require measurement')
Path(a.output).write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k not in ('model','durations_s')}))
