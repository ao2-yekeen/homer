"""Project millimetre ToF samples using measured optical extrinsics."""
import cv2
import numpy as np

def transform(points, matrix):
    m = np.asarray(matrix, dtype=float)
    if m.shape != (4, 4) or not np.isfinite(m).all() or not np.allclose(m[3], [0, 0, 0, 1]):
        raise ValueError('invalid rigid transform')
    if not np.allclose(m[:3, :3].T @ m[:3, :3], np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(m[:3, :3]), 1):
        raise ValueError('rotation must be orthonormal')
    return points @ m[:3, :3].T + m[:3, 3]

def estimate(depth_mm, mask, c, view):
    if depth_mm.shape != tuple(c['tof_shape']) or mask.shape != tuple(c['rgb_shape']):
        raise ValueError('uncalibrated image dimensions')
    yy, xx = np.indices(depth_mm.shape)
    d = depth_mm.ravel().astype(float) * .001
    uv = np.column_stack((xx.ravel(), yy.ravel())).astype(float)
    rays = cv2.undistortPoints(uv.reshape(-1, 1, 2), np.array(c['tof_K'], float), np.array(c['tof_D'], float)).reshape(-1, 2)
    rays = np.column_stack((rays, np.ones(len(rays))))
    if c['depth_semantics'] == 'radial':
        rays /= np.linalg.norm(rays, axis=1)[:, None]
    elif c['depth_semantics'] != 'optical_z':
        raise ValueError('ToF range interpretation unvalidated')
    valid = np.isfinite(d) & (d >= c['depth_min_m']) & (d <= c['depth_max_m'])
    points = rays[valid] * d[valid, None]
    rgb = transform(points, c['tof_to_rgb'])
    points, rgb = points[rgb[:, 2] > 0], rgb[rgb[:, 2] > 0]
    if len(rgb) < c['min_samples']:
        raise ValueError('insufficient valid depth')
    projected, _ = cv2.projectPoints(rgb, np.zeros(3), np.zeros(3), np.array(c['rgb_K'], float), np.array(c['rgb_D'], float))
    pix = np.rint(projected.reshape(-1, 2)).astype(int)
    h, w = mask.shape
    inside = (pix[:, 0] >= 0) & (pix[:, 0] < w) & (pix[:, 1] >= 0) & (pix[:, 1] < h)
    pix, points = pix[inside], points[inside]
    # Erosion avoids mixed pixels at the silhouette boundary.
    clean_mask = cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8))
    points = points[clean_mask[pix[:, 1], pix[:, 0]] > 0]
    base = transform(points, c['views'][view]['tof_to_base'])
    if len(base) < c['min_samples']:
        raise ValueError('insufficient object depth')
    floor = base[:, 2] <= c['floor_reject_m']
    if floor.mean() > c['max_floor_fraction']:
        raise ValueError('floor contamination')
    base = base[~floor]
    if len(base) < c['min_samples'] or np.ptp(base[:, 0]) > c['max_object_depth_span_m']:
        raise ValueError('background contamination or insufficient depth')
    if np.any(base[:, 0] <= 0):
        raise ValueError('object behind robot')
    # Nearest reliable surface; uncertainty must include sampling and unseen surface error.
    clearance = float(np.quantile(base[:, 0], .05) - c['front_edge_m'])
    centre = np.median(base, axis=0)
    return clearance, float(np.arctan2(centre[1], centre[0]))

class Identity:
    def __init__(self):
        self.box = None
        self.identity = 0

    def select(self, boxes, width, height):
        if not boxes:
            return None
        if self.box is None:
            index = min(range(len(boxes)), key=lambda i: ((boxes[i][0]+boxes[i][2])/2-width/2)**2 + ((boxes[i][1]+boxes[i][3])/2-height/2)**2)
            self.identity += 1
        else:
            def iou(b):
                a = self.box
                intersection = max(0, min(a[2], b[2])-max(a[0], b[0])) * max(0, min(a[3], b[3])-max(a[1], b[1]))
                return intersection / max(1, (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection)
            matches = [i for i, b in enumerate(boxes) if iou(b) >= .3]
            if len(matches) != 1:
                raise ValueError('ambiguous identity')
            index = matches[0]
        self.box = boxes[index]
        return index
