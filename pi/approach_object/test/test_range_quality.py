import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1]))
import numpy as np
import pytest
from approach_object.range_quality import stationary_range


def data():
    rng = np.random.default_rng(4)
    d = 600 + rng.normal(0, 3, (20, 16, 16))
    return d, np.full_like(d, 80), np.ones((16, 16), bool), np.arange(20)*.1


def test_repeatable_surface_keeps_front_clearance_unknown():
    r = stationary_range(*data(), confidence_min=30)
    assert r['repeatable']
    assert r['sensor_range_m'] == pytest.approx(.6, abs=.002)
    assert r['front_clearance_m'] is None and not r['motion_authority']


def test_weak_return_cannot_be_averaged_into_distance():
    d, c, m, t = data()
    c[:] = 8
    r = stationary_range(d, c, m, t, confidence_min=30)
    assert not r['repeatable'] and r['sensor_range_m'] is None


def test_temporal_noise_cannot_be_hidden_by_averaging():
    d, c, m, t = data()
    d[::2] += 350
    r = stationary_range(d, c, m, t, confidence_min=30)
    assert not r['repeatable'] and r['stable_pixels'] == 0


def test_two_surfaces_are_rejected_instead_of_choosing_convenient_cluster():
    d, c, m, t = data()
    d[:, :, 8:] += 500
    r = stationary_range(d, c, m, t, confidence_min=30)
    assert not r['repeatable'] and 'spread' in r['reason']


def test_return_loss_in_one_frame_is_rejected():
    d, c, m, t = data()
    c[10] = 0
    r = stationary_range(d, c, m, t, confidence_min=30)
    assert not r['repeatable'] and 'lost' in r['reason']


@pytest.mark.parametrize('bad', ['duplicate', 'unordered', 'gap'])
def test_invalid_frame_history_is_rejected(bad):
    d, c, m, t = data()
    if bad == 'duplicate': t[10] = t[9]
    elif bad == 'unordered': t[10] = 0
    else: t[10:] += 2
    assert not stationary_range(d, c, m, t, confidence_min=30)['repeatable']


def test_invalid_depth_and_dimensions_are_not_accepted():
    d, c, m, t = data()
    d[:] = np.nan
    assert not stationary_range(d, c, m, t, confidence_min=30)['repeatable']
    with pytest.raises(ValueError):
        stationary_range(d, c, m[:2], t, confidence_min=30)
