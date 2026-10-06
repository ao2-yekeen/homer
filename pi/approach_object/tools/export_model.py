#!/usr/bin/env python3
"""Workstation export; no camera or motor access. Run in the pinned environment."""
import hashlib
import json
import os
from pathlib import Path
import importlib.metadata as md
from ultralytics import YOLO

pnnx = Path('pnnx')
pnnx_sha = 'f78fbff9911cc0502c14ff8b852b0c9b4d20000d34bc161916ef851b432ad8b9'
if not pnnx.is_file() or hashlib.sha256(pnnx.read_bytes()).hexdigest() != pnnx_sha:
    raise ValueError('place checksum-pinned PNNX 20250912 Linux binary in the export directory')
os.environ['PATH'] = str(Path.cwd()) + os.pathsep + os.environ['PATH']
model = YOLO('yolo11n-seg.pt')
source_sha = hashlib.sha256(Path('yolo11n-seg.pt').read_bytes()).hexdigest()
if source_sha != '55ed65c56c91713d23e8402371c6c49a6fd84f257f7dce452e8d70e41dcbe152':
    raise ValueError('source weights differ from checked YOLO11n-seg revision')
root = Path(model.export(format='ncnn', imgsz=320, half=False))
manifest = {'model': 'yolo11n-seg', 'imgsz': 320, 'sha256': {}, 'dependencies': {p: md.version(p) for p in ('ultralytics', 'torch', 'torchvision', 'ncnn', 'numpy', 'opencv-python')}}
for p in sorted(root.iterdir()):
    if p.is_file() and p.name != 'manifest.json':
        manifest['sha256'][p.name] = hashlib.sha256(p.read_bytes()).hexdigest()
manifest['pnnx_sha256'] = pnnx_sha
manifest['pnnx_release'] = '20250912'
manifest['source_sha256'] = hashlib.sha256(Path('yolo11n-seg.pt').read_bytes()).hexdigest()
(root / 'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
print(root.resolve())
