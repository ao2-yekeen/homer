"""Stationary ToF repeatability checks. Passing is not proof of metric accuracy.

Masks are in native ToF coordinates. No recognition, camera registration,
robot transform, ROS publishers or command authority is provided here.
"""
import warnings
import cv2
import numpy as np


def stationary_range(depth_mm, confidence, mask, stamps, *, confidence_min,
                     min_frames=10, min_pixels=24, min_coverage=.5,
                     min_valid_fraction=.8, temporal_span_m=.10,
                     spatial_span_m=.12, max_frame_gap_s=.5):
    """Check a stationary target in distinct, ordered acquisition frames.

    Thresholds are experimental policy, not a manufacturer accuracy spec.
    Preserve spatial spread rather than selecting a convenient depth cluster.
    """
    depth = np.asarray(depth_mm, dtype=float) * .001
    quality = np.asarray(confidence, dtype=float)
    mask = np.asarray(mask, dtype=np.uint8)
    stamps = np.asarray(stamps, dtype=float)
    if depth.ndim != 3 or quality.shape != depth.shape or mask.shape != depth.shape[1:]:
        raise ValueError('depth/confidence/mask dimensions differ')
    if stamps.shape != (len(depth),) or not np.isfinite(stamps).all():
        raise ValueError('invalid acquisition timestamps')
    if not np.isfinite(confidence_min) or confidence_min < 0:
        raise ValueError('invalid confidence threshold')
    if (min_frames < 2 or min_pixels < 1 or not 0 < min_coverage <= 1
            or not 0 < min_valid_fraction <= 1 or temporal_span_m <= 0
            or spatial_span_m <= 0 or max_frame_gap_s <= 0):
        raise ValueError('invalid quality policy')
    result = dict(repeatable=False, sensor_range_m=None, front_clearance_m=None,
                  motion_authority=False, frames=len(depth), reason='',
                  confidence_min=float(confidence_min))
    if len(depth) < min_frames:
        return dict(result, reason='insufficient distinct frames')
    if np.any(np.diff(stamps) <= 0) or np.any(np.diff(stamps) > max_frame_gap_s):
        return dict(result, reason='duplicate, unordered or interrupted acquisition frames')
    interior = cv2.erode(mask, np.ones((3, 3), np.uint8)) > 0
    count = int(interior.sum())
    if count < min_pixels:
        return dict(result, reason='insufficient mask interior')
    values = depth[:, interior]
    valid = (np.isfinite(values) & (values >= .05) & (values <= 4)
             & np.isfinite(quality[:, interior])
             & (quality[:, interior] >= confidence_min))
    accepted = np.where(valid, values, np.nan)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        medians = np.nanmedian(accepted, axis=0)
        lo, hi = np.nanpercentile(accepted, [5, 95], axis=0)
    stable = ((valid.mean(axis=0) >= min_valid_fraction)
              & np.isfinite(medians) & ((hi - lo) <= temporal_span_m))
    result.update(mask_interior_pixels=count, stable_pixels=int(stable.sum()),
                  stable_coverage=float(stable.mean()),
                  quality_valid_fraction=float(valid.mean()),
                  per_frame_valid_fraction=valid.mean(axis=1).tolist(),
                  acquisition_duration_s=float(stamps[-1]-stamps[0]))
    if stable.sum() < min_pixels or stable.mean() < min_coverage:
        return dict(result, reason='insufficient reliable, temporally stable object pixels')
    if np.any(valid.mean(axis=1) < min_coverage):
        return dict(result, reason='object return lost in one or more frames')
    surface = medians[stable]
    p5, p50, p95 = np.percentile(surface, [5, 50, 95])
    result.update(surface_percentiles_m=[float(p5), float(p50), float(p95)],
                  spatial_span_m=float(p95-p5))
    if p95-p5 > spatial_span_m:
        return dict(result, reason='object depth spread too large; contamination or poor return')
    return dict(result, repeatable=True, sensor_range_m=float(p50),
                reason='repeatable stationary sensor range; registration and physical accuracy unverified')
