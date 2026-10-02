"""Validation shared by launch files, nodes, and hardware-independent tests."""
import math
from pathlib import Path
import yaml


def load_geometry(path):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or data.get('verified') is not True:
        raise ValueError('Geometry is unverified: measure laser origin and footprint, then set verified: true')
    if not str(data.get('measurement_source', '')).strip():
        raise ValueError('Geometry requires measurement_source')
    laser = data.get('laser', {})
    for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        value = laser.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('Invalid laser.' + key)
    if abs(laser['roll']) > 0.05 or abs(laser['pitch']) > 0.05:
        raise ValueError('Planar SLAM requires a level laser (roll/pitch within 0.05 radians)')
    points = data.get('footprint', [])
    if len(points) < 3 or any(len(p) != 2 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in p) for p in points):
        raise ValueError('Footprint requires at least three finite [x, y] vertices')
    area = sum(a[0]*b[1] - b[0]*a[1] for a, b in zip(points, points[1:] + points[:1]))
    if abs(area) < 1e-6:
        raise ValueError('Footprint has zero area')
    return data


def map_prefix(value):
    path = Path(value).expanduser().resolve()
    if path.suffix in ('.posegraph', '.data'):
        path = path.with_suffix('')
    for suffix in ('.posegraph', '.data'):
        if not Path(str(path) + suffix).is_file():
            raise ValueError('Missing serialized map file: ' + str(path) + suffix)
    return path


def age_seconds(now_ns, stamp):
    return (now_ns - stamp.sec * 1_000_000_000 - stamp.nanosec) / 1e9


def freshness(age, limit, future_tolerance=0.05):
    if age is None:
        return 'missing'
    if age < -future_tolerance:
        return 'future timestamp: check clocks'
    return 'fresh' if age <= limit else 'stale'
