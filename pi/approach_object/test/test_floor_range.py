import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1]))
import numpy as np
import pytest
from approach_object.floor_range import fit_plane, intersect_ray


def test_plane_fit_tolerates_isolated_bad_depth():
    rng = np.random.default_rng(7)
    xz = rng.uniform(-.5, .5, (400, 2))
    points = np.column_stack((xz[:, 0], .4+rng.normal(0, .002, 400), xz[:, 1]+1))
    points[:50] = rng.uniform(0, 2, (50, 3))
    n, d, quality = fit_plane(points)
    hit = intersect_ray(n, d, [0, 0, 0], [0, .4, 1])
    assert hit == pytest.approx([0, .4, 1], abs=.01)
    assert quality['inlier_fraction'] > .8


def test_unstructured_noise_is_not_a_floor():
    rng = np.random.default_rng(3)
    with pytest.raises(ValueError, match='supported'):
        fit_plane(rng.uniform(-1, 1, (400, 3)))


def test_collinear_samples_rejected():
    t = np.linspace(.1, 1, 200)
    with pytest.raises(ValueError):
        fit_plane(np.column_stack((t, t, t)))


@pytest.mark.parametrize('ray', [[1, 0, 0], [0, -.4, 1], [0, 0, 0]])
def test_parallel_behind_and_zero_rays_rejected(ray):
    with pytest.raises(ValueError):
        intersect_ray([0, 1, 0], -.4, [0, 0, 0], ray)
