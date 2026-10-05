#!/usr/bin/env python3
"""Report source-grounded camera mounts and reference ray checks. No motion."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from approach_object.urdf_geometry import camera_geometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, default=PACKAGE.parents[1]/'simulation/urdf/home_mobile_manipulator_v6_real_base_layout.urdf')
    parser.add_argument('--pitch-down-deg', required=True, type=float, help='physical optical pitch; servo command is not pitch')
    parser.add_argument('--reference-clearance-m', type=float)
    parser.add_argument('--object-height-m', type=float)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists')
    for value in (args.reference_clearance_m, args.object_height_m):
        if value is not None and (not np.isfinite(value) or value < 0):
            parser.error('reference dimensions must be finite nonnegative metres')
    report = camera_geometry(args.urdf, args.pitch_down_deg)
    report['urdf_sha256'] = hashlib.sha256(args.urdf.read_bytes()).hexdigest()
    report['urdf_path'] = str(args.urdf)
    report['labels'] = 'URDF GEOMETRY / OPTICAL POSE UNQUALIFIED'
    lens = np.array(report['tof_to_base'])[:3, 3]
    report['model_tof_origin_height_m'] = float(lens[2])
    report['model_tof_origin_behind_front_m'] = float(report['front_edge_m']-lens[0])
    if args.reference_clearance_m is not None:
        reference = np.array([report['front_edge_m']+args.reference_clearance_m, 0., 0.])
        report['reference_check'] = {'clearance_m': args.reference_clearance_m,
            'assumption': 'reference point at robot centreline; not inferred measured clearance',
            'predicted_floor_ray_m': float(np.linalg.norm(reference-lens))}
        if args.object_height_m is not None:
            reference[2] = args.object_height_m
            report['reference_check'].update(object_height_m=args.object_height_m,
                predicted_top_ray_m=float(np.linalg.norm(reference-lens)))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
