"""Plane fit and ray intersection for stationary floor-contact diagnostics.

Caller must establish that samples describe a level floor and that the visible
object contact lies on it. Neither semantic assertion follows from RANSAC.
"""
import numpy as np


def fit_plane(points, *, residual_m=.025, min_inlier_fraction=.6,
              min_inliers=100, iterations=300):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('invalid plane samples')
    if (residual_m <= 0 or not 0 < min_inlier_fraction <= 1
            or min_inliers < 3 or iterations < 1):
        raise ValueError('invalid plane policy')
    if len(points) < min_inliers:
        raise ValueError('insufficient floor samples')
    rng = np.random.default_rng(42)
    best = np.zeros(len(points), dtype=bool)
    for _ in range(iterations):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b-a, c-a)
        length = np.linalg.norm(normal)
        if length < 1e-8:
            continue
        normal /= length
        inliers = np.abs((points-a) @ normal) <= residual_m
        if inliers.sum() > best.sum():
            best = inliers
    if best.sum() < min_inliers or best.mean() < min_inlier_fraction:
        raise ValueError('no sufficiently supported floor plane')
    centre = points[best].mean(axis=0)
    _, singular, vectors = np.linalg.svd(points[best]-centre, full_matrices=False)
    if singular[1] < .05:
        raise ValueError('floor samples have insufficient two-dimensional extent')
    normal = vectors[-1]
    offset = -float(centre @ normal)
    residuals = np.abs(points @ normal + offset)
    inliers = residuals <= residual_m
    if inliers.sum() < min_inliers or inliers.mean() < min_inlier_fraction:
        raise ValueError('refined floor plane lacks support')
    return normal, offset, dict(samples=len(points), inliers=int(inliers.sum()),
                               inlier_fraction=float(inliers.mean()),
                               residual_p95_m=float(np.percentile(residuals[inliers], 95)))


def intersect_ray(normal, offset, origin, direction):
    normal, origin, direction = [np.asarray(v, dtype=float) for v in (normal, origin, direction)]
    if any(v.shape != (3,) or not np.isfinite(v).all() for v in (normal, origin, direction)) or not np.isfinite(offset):
        raise ValueError('invalid plane or ray')
    length = np.linalg.norm(direction)
    if length < 1e-8:
        raise ValueError('zero-length ray')
    direction = direction / length
    denominator = float(normal @ direction)
    if abs(denominator) < .05:
        raise ValueError('ray too close to parallel with floor')
    distance = -(float(normal @ origin)+offset)/denominator
    if not .05 <= distance <= 4:
        raise ValueError('floor contact outside supported range')
    return origin + distance*direction
