#!/usr/bin/env python3
"""Offline stationary range check using an explicitly supplied native ToF mask.

This diagnostic never supplies robot clearance or authorises motion. A mask
must describe the same stationary object throughout the burst; inspect RGB
frames for movement. No automatic RGB/ToF registration is claimed.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from approach_object.range_quality import stationary_range


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture', required=True)
    p.add_argument('--tof-mask', required=True)
    p.add_argument('--confidence-min', required=True, type=float,
                   help='Experimental raw SDK threshold, not an accuracy guarantee')
    p.add_argument('--max-frame-gap-s', type=float, default=.5,
                   help='Stationary diagnostic only; larger gaps never qualify moving operation')
    a = p.parse_args()
    root = Path(a.capture)
    folders = sorted(root.glob('frame-*'))
    if not folders:
        p.error('no burst frame directories')
    mask = cv2.imread(a.tof_mask, cv2.IMREAD_GRAYSCALE)
    if mask is None:
        p.error('native ToF mask unreadable')
    depth = np.stack([np.load(f/'depth-mm.npy', allow_pickle=False) for f in folders])
    confidence = np.stack([np.load(f/'confidence.npy', allow_pickle=False) for f in folders])
    metadata = [json.loads((f/'capture.json').read_text()) for f in folders]
    if any(m['tof_frame'] != metadata[0]['tof_frame'] for m in metadata):
        p.error('ToF coordinate frame changed during burst')
    result = stationary_range(depth, confidence, mask > 0,
                              [m['depth_stamp'] for m in metadata],
                              confidence_min=a.confidence_min,
                              max_frame_gap_s=a.max_frame_gap_s)
    result.update(mask_source=str(a.tof_mask), diagnostic_only=True,
                  quality_policy=dict(min_frames=10, min_pixels=24, min_coverage=.5,
                                      min_valid_fraction=.8, temporal_span_m=.10,
                                      spatial_span_m=.12, max_frame_gap_s=a.max_frame_gap_s))
    (root/'range-quality.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
