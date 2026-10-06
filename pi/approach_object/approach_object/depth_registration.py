"""Depth surface visibility and conservative per-frame RGB/IR registration.

Image translation is a provisional registration refinement, not a new rigid
calibration. It is estimated from scene structure outside the target mask.
"""
import cv2
import numpy as np

from .geometry import transform


def camera_samples(depth_mm, calibration):
    h, w = depth_mm.shape
    y, x = np.indices((h, w))
    uv = np.column_stack((x.ravel(), y.ravel())).astype(float)
    rays = cv2.undistortPoints(uv[:, None], np.asarray(calibration['tof_K'], float),
                               np.asarray(calibration['tof_D'], float)).reshape(-1, 2)
    rays = np.column_stack((rays, np.ones(len(rays))))
    if calibration['depth_semantics'] == 'radial':
        rays /= np.linalg.norm(rays, axis=1)[:, None]
    elif calibration['depth_semantics'] != 'optical_z':
        raise ValueError('unknown depth semantics')
    ranges = np.asarray(depth_mm, float).ravel() * .001
    points = rays * ranges[:, None]
    rgb_points = transform(points, calibration['tof_to_rgb'])
    valid = (np.isfinite(points).all(axis=1) & np.isfinite(rgb_points).all(axis=1)
             & (ranges >= calibration.get('depth_min_m', .05))
             & (ranges <= calibration.get('depth_max_m', 4)) & (rgb_points[:, 2] > .01))
    pixels = np.full((len(points), 2), np.nan)
    if valid.any():
        pixels[valid] = cv2.projectPoints(rgb_points[valid], np.zeros(3), np.zeros(3),
                                        np.asarray(calibration['rgb_K'], float),
                                        np.asarray(calibration['rgb_D'], float))[0].reshape(-1, 2)
    return points, rgb_points, pixels, valid


def rasterize(depth_mm, calibration, rgb_shape, offset=(0., 0.)):
    """Rasterize continuous native triangles with perspective-correct depth.

    Never bridge invalid returns or a depth discontinuity. Native indices are
    carried through the depth buffer so interpolated pixels are not counted
    as independent measurements.
    """
    points, rgb_points, pixels, valid = camera_samples(depth_mm, calibration)
    pixels = pixels + np.asarray(offset)
    rh, rw = rgb_shape
    zbuffer = np.full(rh * rw, np.inf)
    owner = np.full(rh * rw, -1, np.int32)
    ids = np.arange(depth_mm.size).reshape(depth_mm.shape)
    a, b, c, d = ids[:-1, :-1], ids[:-1, 1:], ids[1:, :-1], ids[1:, 1:]
    triangles = np.concatenate((np.stack((a, b, c), -1).reshape(-1, 3),
                                np.stack((b, d, c), -1).reshape(-1, 3)))
    triangles = triangles[valid[triangles].all(axis=1)]
    ranges = np.linalg.norm(points, axis=1)
    step = float(calibration.get('registration_triangle_step_m', .05))
    if not np.isfinite(step) or step <= 0:
        raise ValueError('registration_triangle_step_m must be finite and positive')
    triangles = triangles[np.ptp(ranges[triangles], axis=1) <= step]
    xy = pixels[triangles]
    lo = np.maximum(np.ceil(xy.min(axis=1)).astype(int), [0, 0])
    hi = np.minimum(np.floor(xy.max(axis=1)).astype(int), [rw-1, rh-1])
    extent = hi-lo+1
    good = (extent > 0).all(axis=1) & (extent <= 32).all(axis=1)
    triangles, xy, lo, extent = triangles[good], xy[good], lo[good], extent[good]
    if len(triangles):
        v0, v1, v2 = xy[:, 0], xy[:, 1], xy[:, 2]
        denominator = (v1[:, 1]-v2[:, 1])*(v0[:, 0]-v2[:, 0]) + (v2[:, 0]-v1[:, 0])*(v0[:, 1]-v2[:, 1])
        nondegenerate = np.abs(denominator) > 1e-8
        for oy in range(int(extent[:, 1].max())):
            for ox in range(int(extent[:, 0].max())):
                active = nondegenerate & (extent[:, 0] > ox) & (extent[:, 1] > oy)
                t = np.flatnonzero(active)
                x, y = lo[t, 0]+ox, lo[t, 1]+oy
                w0 = ((v1[t, 1]-v2[t, 1])*(x-v2[t, 0]) + (v2[t, 0]-v1[t, 0])*(y-v2[t, 1]))/denominator[t]
                w1 = ((v2[t, 1]-v0[t, 1])*(x-v2[t, 0]) + (v0[t, 0]-v2[t, 0])*(y-v2[t, 1]))/denominator[t]
                weights = np.column_stack((w0, w1, 1-w0-w1))
                inside = (weights >= -1e-6).all(axis=1)
                t, x, y, weights = t[inside], x[inside], y[inside], weights[inside]
                z = 1 / np.sum(weights/rgb_points[triangles[t], 2], axis=1)
                linear = y*rw+x
                # Resolve collisions within this vectorized batch as well.
                order = np.lexsort((z, linear))
                first = np.r_[True, np.diff(linear[order]) != 0] if len(order) else np.empty(0, bool)
                take = order[first]
                linear, z, t, weights = linear[take], z[take], t[take], weights[take]
                nearer = z < zbuffer[linear]
                linear, z, t, weights = linear[nearer], z[nearer], t[nearer], weights[nearer]
                zbuffer[linear] = z
                owner[linear] = triangles[t, np.argmax(weights, axis=1)]
    # Retain isolated measured centers, but they may not overwrite a nearer
    # rendered surface. They still face spatial-support checks downstream.
    indices = np.flatnonzero(valid)
    centers = np.rint(pixels[indices]).astype(int)
    inside = (centers[:, 0] >= 0) & (centers[:, 0] < rw) & (centers[:, 1] >= 0) & (centers[:, 1] < rh)
    indices, centers = indices[inside], centers[inside]
    linear = centers[:, 1]*rw+centers[:, 0]
    z = rgb_points[indices, 2]
    order = np.lexsort((z, linear))
    first = np.r_[True, np.diff(linear[order]) != 0] if len(order) else np.empty(0, bool)
    take = order[first]
    linear, z, indices = linear[take], z[take], indices[take]
    nearer = z < zbuffer[linear]
    zbuffer[linear[nearer]] = z[nearer]
    owner[linear[nearer]] = indices[nearer]
    return owner.reshape(rgb_shape), points, rgb_points, pixels


def point_footprints(depth_mm, calibration, rgb_shape, offset=(0., 0.)):
    """Project bounded native pixel footprints without bridging depth triangles.

    Corners follow the measured centre's depth, not interpolated neighbour
    depths. The z buffer resolves visibility at RGB pixel resolution. Footprints
    approximate a sampling cell, not the optical PSF or registration certainty.
    """
    points, rgb_points, pixels, valid = camera_samples(depth_mm, calibration)
    pixels += np.asarray(offset)
    # Invalid/out-of-range sensor returns remain unknown in the audit ledger.
    points[~valid] = np.nan
    h, w = depth_mm.shape
    rh, rw = rgb_shape
    ids = np.flatnonzero(valid)
    if not len(ids):
        return np.full(rgb_shape, -1, np.int32), points, rgb_points, pixels
    y, x = np.divmod(ids, w)
    corners = np.stack([np.column_stack((x+dx, y+dy))
                        for dx, dy in ((-.5, -.5), (.5, -.5), (.5, .5), (-.5, .5))], axis=1)
    rays = cv2.undistortPoints(corners.reshape(-1, 1, 2).astype(float),
        np.asarray(calibration['tof_K'], float), np.asarray(calibration['tof_D'], float)).reshape(-1, 2)
    rays = np.column_stack((rays, np.ones(len(rays))))
    if calibration['depth_semantics'] == 'radial':
        rays /= np.linalg.norm(rays, axis=1)[:, None]
    corner_points = transform(rays * np.repeat(depth_mm.ravel()[ids]*.001, 4)[:, None], calibration['tof_to_rgb'])
    projected = cv2.projectPoints(corner_points, np.zeros(3), np.zeros(3),
        np.asarray(calibration['rgb_K'], float), np.asarray(calibration['rgb_D'], float))[0].reshape(-1, 4, 2)
    projected += np.asarray(offset)
    good = (corner_points.reshape(-1, 4, 3)[:, :, 2] > .01).all(axis=1) & np.isfinite(projected).all(axis=(1, 2))
    ids, projected = ids[good], projected[good]
    # Clip floating coordinates before integer conversion, including off-screen cells.
    lo = np.ceil(np.clip(projected.min(axis=1), [-1, -1], [rw, rh])).astype(int)
    hi = np.floor(np.clip(projected.max(axis=1), [-1, -1], [rw, rh])).astype(int)
    lo = np.maximum(lo, [0, 0]); hi = np.minimum(hi, [rw-1, rh-1])
    extent = hi-lo+1
    good = (extent > 0).all(axis=1) & (extent <= 32).all(axis=1)
    ids, projected, lo, extent = ids[good], projected[good], lo[good], extent[good]
    owner = np.full(rh*rw, -1, np.int32)
    zbuffer = np.full(rh*rw, np.inf)
    if len(ids):
        for dy in range(int(extent[:, 1].max())):
            for dx in range(int(extent[:, 0].max())):
                active = (extent[:, 0] > dx) & (extent[:, 1] > dy)
                samples = ids[active]
                xy = lo[active]+[dx, dy]
                polygon = projected[active]
                edges = np.roll(polygon, -1, axis=1)-polygon
                relative = xy[:, None, :]-polygon
                cross = edges[:, :, 0]*relative[:, :, 1]-edges[:, :, 1]*relative[:, :, 0]
                inside = (cross >= -1e-8).all(axis=1) | (cross <= 1e-8).all(axis=1)
                samples, xy = samples[inside], xy[inside]
                if not len(samples):
                    continue
                linear = xy[:, 1]*rw+xy[:, 0]
                z = rgb_points[samples, 2]
                order = np.lexsort((samples, z, linear))
                first = np.r_[True, np.diff(linear[order]) != 0]
                take = order[first]
                linear, z, samples = linear[take], z[take], samples[take]
                nearer = z < zbuffer[linear]
                owner[linear[nearer]] = samples[nearer]
                zbuffer[linear[nearer]] = z[nearer]
    return owner.reshape(rgb_shape), points, rgb_points, pixels


def refine_translation(rgb, amplitude, owners, target, calibration):
    """Use background gradient structure, rejecting weak/inconsistent fits."""
    info = dict(status='unavailable', offset_xy_px=[0., 0.],
                method='background_RGB_IR_gradient_translation', qualified=False)
    if amplitude is None:
        return (0., 0.), info | {'reason': 'synchronized infrared image missing'}
    if amplitude.size != int(np.prod(calibration['tof_shape'])):
        return (0., 0.), info | {'reason': 'infrared dimensions differ from calibration'}
    signal = np.log1p(np.maximum(np.nan_to_num(amplitude, nan=0, posinf=0, neginf=0), 0))
    lo, hi = np.percentile(signal, [1, 99])
    signal = np.clip((signal-lo)/max(hi-lo, 1e-6), 0, 1)
    valid = owners >= 0
    image = np.zeros(owners.shape, np.float32)
    image[valid] = signal.ravel()[owners[valid]]
    # Fill only small sampling holes for gradient computation, not depth.
    numerator = cv2.GaussianBlur(image, (7, 7), 1.5)
    denominator = cv2.GaussianBlur(valid.astype(np.float32), (7, 7), 1.5)
    image = numerator / np.maximum(denominator, .001)
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY).astype(np.float32)/255
    def gradient(im):
        im = cv2.GaussianBlur(im, (5, 5), 1)
        return cv2.GaussianBlur(cv2.magnitude(cv2.Sobel(im, cv2.CV_32F, 1, 0),
                                             cv2.Sobel(im, cv2.CV_32F, 0, 1)), (11, 11), 2)
    source, template = gradient(image), gradient(gray)
    limit = float(calibration.get('registration_max_shift_px', 80))
    if not np.isfinite(limit) or not 0 <= limit <= 100:
        raise ValueError('registration_max_shift_px must be between 0 and 100')
    # Exclude the target AND its uncertainty neighborhood from the fit.
    excluded = cv2.dilate(target.astype(np.uint8), np.ones((int(2*limit+1), int(2*limit+1)), np.uint8)) > 0
    # Do not paint an identical black target silhouette into both images:
    # ECC can align that artificial edge instead of real scene structure.
    # Likewise depth coverage boundaries must not become infrared features.
    interior = cv2.GaussianBlur(valid.astype(np.float32), (15, 15), 3) > .9
    usable = ~excluded & interior
    if usable.sum() < 2000 or np.std(source[usable]) < 1e-5 or np.std(template[usable]) < 1e-5:
        return (0., 0.), info | {'reason': 'insufficient background structure'}
    fits = []
    for initial in (-limit*.75, 0., limit*.75):
        warp = np.eye(2, 3, dtype=np.float32)
        warp[1, 2] = initial
        try:
            score, warp = cv2.findTransformECC(template, source, warp, cv2.MOTION_TRANSLATION,
                (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 150, 1e-5),
                (usable.astype(np.uint8)*255), 5)
            if np.isfinite(warp).all() and np.max(np.abs(warp[:, 2])) <= limit:
                fits.append((float(score), warp))
        except cv2.error:
            continue
    if not fits:
        return (0., 0.), info | {'reason': 'background registration did not converge'}
    score, warp = max(fits, key=lambda item: item[0])
    corrected = cv2.warpAffine(source, warp, (source.shape[1], source.shape[0]),
                               flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
    corrected_valid = cv2.warpAffine(usable.astype(np.uint8), warp, (source.shape[1], source.shape[0]),
                                     flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP) > 0
    def correlation(a, b, use):
        if use.sum() < 200 or np.std(a[use]) < 1e-5 or np.std(b[use]) < 1e-5:
            return None
        return float(np.corrcoef(a[use], b[use])[0, 1])
    use = usable & corrected_valid & valid
    before, after = correlation(template, source, use), correlation(template, corrected, use)
    improvements = []
    h, w = target.shape
    for y0, y1, x0, x1 in ((0,h//2,0,w//2),(0,h//2,w//2,w),(h//2,h,0,w//2),(h//2,h,w//2,w)):
        tile = np.zeros_like(use); tile[y0:y1, x0:x1] = True; tile &= use
        b, a = correlation(template, source, tile), correlation(template, corrected, tile)
        if a is not None and b is not None:
            improvements.append(a-b)
    info.update(ecc_score=score, proposed_offset_xy_px=(-warp[:, 2].astype(float)).tolist(),
                correlation_before=before, correlation_after=after,
                tile_correlation_improvements=improvements)
    if before is None or after is None or after < .15 or after-before < .03 or sum(v > .02 for v in improvements) < 2:
        return (0., 0.), info | {'status': 'rejected', 'reason': 'background registration has insufficient independent support'}
    offset = -warp[:, 2].astype(float)
    info.update(status='provisional', offset_xy_px=offset.tolist(),
                reason='per-frame background translation; rigid calibration and physical accuracy remain unverified')
    return tuple(offset), info
