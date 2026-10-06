"""Partial target continuity, ambiguity and loss, without model downloads."""
import cv2
import numpy as np
import pytest

from approach_object.visual_target import VisualTarget


def frame():
    return np.zeros((100, 120, 3), np.uint8)


def mask(x0, y0, x1, y1):
    result = np.zeros((100, 120), bool)
    result[y0:y1, x0:x1] = True
    return result


def test_small_visible_fragment_keeps_identity_despite_low_box_iou():
    tracker = VisualTarget()
    tracker.select(frame(), [mask(10, 10, 80, 90)], 1.)
    # Fragment is less than 0.3 of the original area, failing old box IoU.
    index, result = tracker.select(frame(), [mask(10, 55, 35, 90)], 1.1)
    assert index == 0 and result['selected_identity'] == 1
    assert result['visibility'] == 'partial'


def test_full_object_return_matches_original_extent_after_fragment():
    tracker = VisualTarget()
    full = mask(10, 10, 80, 90)
    tracker.select(frame(), [full], 1.)
    tracker.select(frame(), [mask(10, 55, 35, 90)], 1.1)
    index, result = tracker.select(frame(), [full], 1.2)
    assert index == 0 and result['selected_identity'] == 1
    assert result['visibility'] == 'visible_extent_unknown'
    assert np.array_equal(tracker.mask, full)


def test_frame_boundary_is_supported():
    tracker = VisualTarget()
    _, result = tracker.select(frame(), [mask(0, 30, 30, 80)], 1.)
    assert result['rgb_boundary_clipped'] and result['visibility'] == 'partial'


def test_initial_multiple_objects_require_selection():
    candidates = [mask(5, 20, 30, 70), mask(60, 20, 90, 70)]
    tracker = VisualTarget()
    with pytest.raises(ValueError, match='ambiguous'):
        tracker.select(frame(), candidates, 1.)
    index, _ = tracker.select(frame(), candidates, 1., [60, 20, 90, 70])
    assert index == 1


def test_ambiguity_preserves_memory_then_unique_candidate_resumes():
    tracker = VisualTarget()
    original = mask(20, 20, 70, 80)
    tracker.select(frame(), [original], 1.)
    with pytest.raises(ValueError, match='ambiguous'):
        tracker.select(frame(), [original, mask(25, 25, 65, 75)], 1.1)
    assert tracker.stamp == 1. and np.array_equal(tracker.mask, original)
    _, result = tracker.select(frame(), [original], 1.2)
    assert result['selected_identity'] == 1


def test_no_jump_to_separate_object_or_stale_target():
    tracker = VisualTarget()
    tracker.select(frame(), [mask(5, 20, 30, 70)], 1.)
    with pytest.raises(ValueError, match='association failed'):
        tracker.select(frame(), [mask(70, 20, 100, 70)], 1.1)
    with pytest.raises(ValueError, match='gap'):
        tracker.select(frame(), [mask(5, 20, 30, 70)], 1.6)
    with pytest.raises(ValueError, match='reselection'):
        tracker.select(frame(), [mask(5, 20, 30, 70)], 1.7)


def test_insufficient_fragment_rejected():
    tracker = VisualTarget()
    tracker.select(frame(), [mask(10, 10, 80, 90)], 1.)
    with pytest.raises(ValueError, match='absent'):
        tracker.select(frame(), [mask(10, 10, 12, 12)], 1.1)


def test_optical_flow_follows_camera_translation():
    tracker = VisualTarget()
    image = frame()
    rng = np.random.default_rng(3)
    image[20:80, 20:70] = rng.integers(0, 256, (60, 50, 3), dtype=np.uint8)
    tracker.select(image, [mask(20, 20, 70, 80)], 1.)
    moved = cv2.warpAffine(image, np.array([[1., 0., 12.], [0., 1., 4.]]), (120, 100))
    _, result = tracker.select(moved, [mask(32, 24, 82, 84)], 1.1)
    assert result['flow_inliers'] >= 6
