"""Checked model loading and stride-aware sensor image decoding."""
import hashlib
import importlib.metadata as md
import json
from pathlib import Path
import numpy as np
from .model_pin import MODEL_SHA256, SOURCE_SHA256

def verify_model(directory):
    root = Path(directory)
    manifest = json.loads((root / 'manifest.json').read_text())
    if manifest['model'] != 'yolo11n-seg' or manifest['imgsz'] != 320:
        raise ValueError('model must be YOLO11n-seg at 320')
    if {k:v for k,v in manifest['sha256'].items() if k != 'metadata.yaml'} != MODEL_SHA256 or manifest['source_sha256'] != SOURCE_SHA256:
        raise ValueError('model differs from checked revision')
    for dependency in ('ultralytics', 'ncnn', 'numpy', 'opencv-python'):
        if md.version(dependency) != manifest['dependencies'][dependency]:
            raise ValueError('inference dependency differs from export: ' + dependency)
    for name, digest in manifest['sha256'].items():
        if Path(name).name != name or hashlib.sha256((root/name).read_bytes()).hexdigest() != digest:
            raise ValueError('model checksum mismatch: ' + name)
    if not {'model.ncnn.bin', 'model.ncnn.param', 'metadata.yaml'} <= manifest['sha256'].keys():
        raise ValueError('incomplete model manifest')
    return manifest

def image_array(m, encoding, dtype, channels=1):
    if m.encoding != encoding:
        raise ValueError('unexpected image encoding')
    item = np.dtype(dtype).itemsize
    if m.step < m.width * channels * item or len(m.data) != m.height * m.step:
        raise ValueError('malformed image stride')
    dt = np.dtype(dtype).newbyteorder('>' if m.is_bigendian else '<')
    rows = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)
    pixels = rows[:, :m.width*channels*item].copy().view(dt)
    return pixels.reshape((m.height, m.width, channels) if channels > 1 else (m.height, m.width))
