"""Visual-mask association for stationary diagnostics. No command authority.

YOLOE supplies fresh masks from an arbitrary reference example. Optical flow
predicts where the previous mask moved; containment permits a smaller visible
fragment. Flow is never used to invent a fresh segmentation or depth reading.
"""
import cv2
import numpy as np


class VisualTarget:
    def __init__(self, max_gap_s=.5, min_overlap=.65, min_pixels=24):
        self.max_gap_s = max_gap_s
        self.min_overlap = min_overlap
        self.min_pixels = min_pixels
        self.mask = self.extent = self.gray = self.stamp = None
        self.identity = 1
        self.lost = False

    def _prediction(self, gray):
        points = cv2.goodFeaturesToTrack(self.gray, 100, .02, 5,
                                        mask=self.extent.astype(np.uint8) * 255)
        if points is None or len(points) < 6:
            return self.extent, 0
        forward, status, _ = cv2.calcOpticalFlowPyrLK(self.gray, gray, points, None)
        if forward is None:
            return self.extent, 0
        backward, back_status, _ = cv2.calcOpticalFlowPyrLK(gray, self.gray, forward, None)
        if backward is None:
            return self.extent, 0
        good = ((status.ravel() != 0) & (back_status.ravel() != 0)
                & (np.linalg.norm(points[:, 0] - backward[:, 0], axis=1) < 1.5))
        if good.sum() < 6:
            return self.extent, 0
        matrix, inliers = cv2.estimateAffinePartial2D(points[good, 0], forward[good, 0],
                                                   method=cv2.RANSAC, ransacReprojThreshold=2.)
        if matrix is None or inliers is None or inliers.sum() < 6:
            return self.extent, 0
        scale = np.linalg.norm(matrix[:, 0])
        if not np.isfinite(matrix).all() or not .7 <= scale <= 1.4:
            return self.extent, 0
        predicted = cv2.warpAffine(self.extent.astype(np.uint8), matrix,
                                   (gray.shape[1], gray.shape[0]), flags=cv2.INTER_NEAREST) > 0
        return predicted, int(inliers.sum())

    def select(self, image, masks, stamp, selection_box=None):
        """Return selected index and diagnostics; expired memory requires restart.

        Without an initial selection box, exactly one reference match is required.
        A new candidate must overlap most of its own area with the predicted target.
        Small contained masks can match even when whole-box IoU is very low.
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if not np.isfinite(stamp):
            raise ValueError('invalid tracking timestamp')
        if self.lost:
            raise ValueError('target lost; explicit reselection required')
        if self.stamp is not None and (stamp <= self.stamp or stamp - self.stamp > self.max_gap_s
                                       or gray.shape != self.gray.shape):
            self.lost = True
            raise ValueError('tracking gap or image mode changed; reselection required')
        masks = [np.asarray(mask, dtype=bool) for mask in masks]
        if any(mask.shape != gray.shape for mask in masks):
            raise ValueError('candidate mask dimensions differ from RGB')
        eligible = [i for i, mask in enumerate(masks) if mask.sum() >= self.min_pixels]
        flow_inliers = 0
        if self.mask is None:
            if selection_box is not None:
                x0, y0, x1, y1 = selection_box
                if not (0 <= x0 < x1 <= gray.shape[1] and 0 <= y0 < y1 <= gray.shape[0]):
                    raise ValueError('selection box must lie inside first RGB image')
                region = np.zeros(gray.shape, bool)
                region[int(y0):int(y1), int(x0):int(x1)] = True
                eligible = [i for i in eligible
                            if (masks[i] & region).sum() / masks[i].sum() >= self.min_overlap]
        else:
            predicted, flow_inliers = self._prediction(gray)
            eligible = [i for i in eligible if
                        (masks[i] & predicted).sum() / masks[i].sum() >= self.min_overlap
                        and (masks[i] & predicted).sum() >= self.min_pixels]
        if len(eligible) != 1:
            # Keep the last accepted mask briefly, without updating it from an
            # uncertain frame. A later unique match can resume within max_gap_s.
            if self.mask is None and selection_box is not None:
                self.lost = True  # first-frame coordinates cannot seed a later frame
            raise ValueError('ambiguous visual target' if eligible else 'visual target absent or association failed')
        chosen = eligible[0]
        mask = masks[chosen]
        clipped = bool(mask[0].any() or mask[-1].any() or mask[:, 0].any() or mask[:, -1].any())
        previous_area = int(self.mask.sum()) if self.mask is not None else int(mask.sum())
        fragment = self.mask is not None and mask.sum() < .7 * previous_area
        self.mask = mask.copy()  # only this fresh segmentation supplies depth
        self.extent = (mask.copy() if self.extent is None else predicted | mask)
        self.gray, self.stamp = gray, float(stamp)
        return chosen, dict(selected_identity=self.identity,
                            visibility='partial' if clipped or fragment else 'visible_extent_unknown',
                            rgb_boundary_clipped=clipped, flow_inliers=flow_inliers,
                            mask_source='fresh_visual_prompt_segmentation')
