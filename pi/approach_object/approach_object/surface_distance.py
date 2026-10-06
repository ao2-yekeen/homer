"""RGB-guided native ToF membership and independent measurement reliability.

The graph probabilities describe this model, not calibrated physical certainty.
No object class, size, expected distance or dominant depth interval is used.
"""
import cv2
import inspect
import numpy as np
from scipy.sparse import coo_matrix, diags
from scipy.sparse.linalg import cg

# Ubuntu's Pi package uses ``tol``; newer SciPy versions use ``rtol``.
_CG_TOLERANCE_KEY = 'rtol' if 'rtol' in inspect.signature(cg).parameters else 'tol'

# Per-native-sample audit codes. Uncertain membership is retained separately.
STATES = {0: 'invalid_depth', 1: 'outside_rgb', 2: 'not_visible_or_uncovered',
          3: 'background', 4: 'uncertain_membership', 5: 'target_unreliable_depth',
          6: 'target_distance_support'}


def graph_membership(indices, points, prior, shape, config):
    """Solve a soft-seeded random-walker Laplacian on visible native samples.

    RGB footprint overlap supplies unary evidence. Four-connected native edges
    combine range and mask boundaries. A measured noise scale may be supplied;
    otherwise the observed median neighbour difference is an explicitly
    uncalibrated geometry scale. Missing samples are never filled.
    """
    n = len(indices)
    if not n:
        return np.empty(0), dict(status='empty', converged=False)
    strength = float(config.get('graph_rgb_strength', 2.))
    tolerance = float(config.get('graph_solver_rtol', 1e-5))
    iterations = int(config.get('graph_solver_maxiter', 300))
    if not np.isfinite([strength, tolerance]).all() or strength <= 0 or not 0 < tolerance < 1 or iterations < 1:
        raise ValueError('invalid graph solver settings')
    lookup = np.full(shape, -1, np.int32)
    lookup.ravel()[indices] = np.arange(n)
    aa, bb = [], []
    for a, b in ((lookup[:, :-1], lookup[:, 1:]), (lookup[:-1], lookup[1:])):
        good = (a >= 0) & (b >= 0)
        aa.append(a[good]); bb.append(b[good])
    a, b = np.concatenate(aa), np.concatenate(bb)
    ranges = np.linalg.norm(points, axis=1)
    delta = np.abs(ranges[a]-ranges[b])
    supplied = config.get('tof_noise_sigma_m')
    scale = float(supplied) if supplied is not None else (float(np.median(delta)) if len(delta) else 0.)
    if not np.isfinite(scale) or scale < 0 or (supplied is not None and scale == 0):
        raise ValueError('tof_noise_sigma_m must be finite and positive')
    # Machine floor only; no imported sensor noise constant.
    scale = max(scale, np.finfo(float).eps * max(float(ranges.max()), 1.))
    weights = np.exp(-.5*np.minimum(delta/scale, 40.)**2) * np.exp(-4.*(prior[a]-prior[b])**2)
    degree = np.bincount(np.r_[a, b], weights=np.r_[weights, weights], minlength=n)
    laplacian = coo_matrix((-np.r_[weights, weights], (np.r_[a, b], np.r_[b, a])), shape=(n, n)).tocsr()
    matrix = laplacian + diags(degree+strength)
    probability, code = cg(matrix, strength*prior, x0=prior, **{_CG_TOLERANCE_KEY: tolerance},
                           atol=0., maxiter=iterations, M=diags(1./(degree+strength)))
    if code != 0 or not np.isfinite(probability).all():
        # Never silently publish a failed solve as a refined segmentation.
        return prior.copy(), dict(status='failed_using_rgb_prior', converged=False, solver_code=int(code))
    return np.clip(probability, 0, 1), dict(status='converged', converged=True,
        method='soft_seeded_random_walker', nodes=n, edges=len(a),
        geometry_scale_m=scale, geometry_scale_source='supplied_sensor_noise' if supplied is not None else 'uncalibrated_observed_neighbour_difference')


def segment_object(mask, depth, owners, points, projected, reliable, config):
    """Return full membership, distance support, audit and soft evidence maps."""
    shape = depth.shape
    finite = np.isfinite(points).all(axis=1) & (np.linalg.norm(points, axis=1) > 0)
    rh, rw = mask.shape
    inside = finite & np.isfinite(projected).all(axis=1) & (projected[:, 0] >= 0) & (projected[:, 0] < rw) & (projected[:, 1] >= 0) & (projected[:, 1] < rh)
    visible = owners >= 0
    count = np.bincount(owners[visible], minlength=depth.size)
    margin = int(config.get('association_perturbation_px', 1))
    if not 0 <= margin <= 32:
        raise ValueError('association_perturbation_px must be between 0 and 32')
    kernel = np.ones((2*margin+1, 2*margin+1), np.uint8)
    # Translation sensitivity envelope, not a calibrated confidence interval.
    # Erosion only supplies strong evidence; it no longer deletes membership.
    mask_float = mask.astype(np.float32)
    soft = cv2.blur(mask_float, kernel.shape, borderType=cv2.BORDER_REPLICATE)
    lower = cv2.erode(mask_float, kernel, borderType=cv2.BORDER_REPLICATE)
    upper = cv2.dilate(mask_float, kernel, borderType=cv2.BORDER_REPLICATE)
    def overlap(image):
        return np.bincount(owners[visible], weights=image[visible], minlength=depth.size)
    indices = np.flatnonzero(count > 0)
    prior = overlap(soft)[indices]/count[indices]
    lower_prior = overlap(lower)[indices]/count[indices]
    upper_prior = overlap(upper)[indices]/count[indices]
    probability, graph = graph_membership(indices, points[indices], prior, shape, config)
    low, high = float(config.get('membership_background_max', .25)), float(config.get('membership_target_min', .75))
    if not 0 <= low < .5 < high <= 1:
        raise ValueError('membership thresholds must straddle 0.5')
    membership = probability >= .5
    certain = (probability >= high) & (lower_prior >= high)
    # Local second differences flag isolated inconsistent measurements while
    # allowing a sloping surface. Use the least residual among available axes
    # so one edge across a fold does not invalidate a coherent adjoining face.
    ranges = np.linalg.norm(points, axis=1).reshape(shape)
    residual = np.full(shape, np.inf)
    for axis in (0, 1):
        previous = np.roll(ranges, 1, axis); following = np.roll(ranges, -1, axis)
        value = np.abs(previous-2*ranges+following)/np.sqrt(6.)
        boundary = [slice(None), slice(None)]
        boundary[axis] = [0, -1]
        value[tuple(boundary)] = np.inf
        value[~np.isfinite(value)] = np.inf
        residual = np.minimum(residual, value)
    available = residual[np.isfinite(residual)]
    noise = config.get('tof_noise_sigma_m')
    if noise is not None:
        if not np.isfinite(float(noise)) or float(noise) <= 0:
            raise ValueError('tof_noise_sigma_m must be finite and positive')
        residual_limit = 6.*float(noise)
        residual_source = 'supplied_sensor_noise'
    elif len(available):
        median = float(np.median(available))
        mad = float(np.median(np.abs(available-median)))
        residual_limit = median + 6.*1.4826*mad
        residual_source = 'uncalibrated_observed_local_residual'
    else:
        residual_limit = 0.
        residual_source = 'no_local_geometry'
    residual_limit = max(residual_limit, np.finfo(float).eps*max(float(np.nanmax(ranges)) if np.isfinite(ranges).any() else 1., 1.))
    local_reliable = np.isfinite(residual) & (residual <= residual_limit)
    support = certain & reliable.ravel()[indices] & local_reliable.ravel()[indices]
    # Require independent local measurement support, without a global range gate.
    grid = np.zeros(shape, np.uint8)
    grid.ravel()[indices[support]] = 1
    neighbours = cv2.filter2D(grid, cv2.CV_32F, np.ones((3, 3), np.float32), borderType=cv2.BORDER_CONSTANT)
    support &= neighbours.ravel()[indices] >= 3
    native = np.zeros(shape, np.uint8); native.ravel()[indices[membership]] = 255
    accepted = np.zeros(shape, np.uint8); accepted.ravel()[indices[support]] = 255
    state = np.zeros(depth.size, np.uint8)
    state[finite] = 1; state[inside] = 2
    state[indices] = 4
    state[indices[(probability <= low) & (upper_prior <= low)]] = 3
    state[indices[certain]] = 5
    state[indices[support]] = 6
    probabilities = np.full(depth.size, np.nan, np.float32)
    probabilities[indices] = probability
    rgb_prior = np.full(depth.size, np.nan, np.float32); rgb_prior[indices] = prior
    info = dict(membership_graph=graph, sample_audit_counts={name: int((state == code).sum()) for code, name in STATES.items()},
                membership_uncertain_samples=int((state == 4).sum()),
                local_geometry_reliability=dict(residual_limit_m=residual_limit, source=residual_source,
                    criterion='minimum available axial second difference; heuristic, not a sensor accuracy bound'),
                association_sensitivity=dict(translation_radius_px=margin,
                    interpretation='configurable projection sensitivity envelope; not measured calibration uncertainty'))
    return indices, membership, support, native, accepted, info, dict(
        membership_probability=probabilities.reshape(shape), rgb_membership_prior=rgb_prior.reshape(shape),
        sample_state=state.reshape(shape), depth_reliable=reliable.astype(np.uint8),
        local_geometry_reliable=local_reliable.astype(np.uint8),
        local_geometry_residual_m=np.where(np.isfinite(residual), residual, np.nan))


def select_surface(indices, points, candidate, shape, config):
    """Report median raw range across reliable target returns, including faces.

    Membership/reliability must be decided upstream. Depth multimodality and
    large extent alone are not evidence of different object identities.
    """
    minimum = max(1, int(config.get('min_samples', 12)))
    ranges = np.linalg.norm(points, axis=1)
    kept = np.asarray(candidate, bool) & np.isfinite(ranges) & (ranges > 0)
    info = dict(surface_samples=int(kept.sum()), distance_cm=None, visible_surface_range_m=None,
                surface_depth_spread_cm=None, distance_status='unavailable',
                distance_origin='tof_camera', depth_statistic='median',
                distance_definition='median Euclidean range to the visible surface')
    if kept.sum() < minimum:
        return np.zeros(len(indices), bool), info | {'surface_samples': 0,
            'surface_range_reason': 'insufficient reliable target measurements'}
    median = float(np.median(ranges[kept]))
    p5, p95 = np.percentile(ranges[kept], [5, 95])
    info.update(distance_cm=float(median*100), visible_surface_range_m=float(median),
                surface_depth_spread_cm=float((p95-p5)*100),
                surface_range_percentiles_cm=[float(v*100) for v in (p5, median, p95)],
                distance_status='provisional',
                surface_range_reason='reliable RGB-associated raw measurements; physical accuracy unverified')
    return kept, info
