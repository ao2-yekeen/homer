from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import pytest
from approach_object.urdf_geometry import camera_geometry

URDF = Path(__file__).resolve().parents[3]/'simulation/urdf/home_mobile_manipulator_v6_real_base_layout.urdf'


def test_neutral_mount_height_plate_front_and_optical_axes():
    result = camera_geometry(URDF, 0)
    pose = np.array(result['tof_to_base'])
    np.testing.assert_allclose(pose[:3, 3], [.025, -.020, .6245])
    np.testing.assert_allclose(pose[:3, 2], [1, 0, 0])
    np.testing.assert_allclose(pose[:3, 0], [0, -1, 0])
    assert result['front_edge_m'] == pytest.approx(.15)
    assert result['physical_accuracy_verified'] is False


def test_tilt_moves_lens_about_pivot_not_in_place():
    result = camera_geometry(URDF, 36)
    pose = np.array(result['tof_to_base'])
    pitch = np.deg2rad(36)
    np.testing.assert_allclose(pose[:3, 3], [-.045+.07*np.cos(pitch), -.02, .6245-.07*np.sin(pitch)])
    np.testing.assert_allclose(pose[:3, 2], [np.cos(pitch), 0, -np.sin(pitch)])
    np.testing.assert_allclose(np.array(result['tof_to_rgb'])[:3, 3], [.04, 0, 0], atol=1e-12)


@pytest.mark.parametrize('pitch', [80, -80, float('nan')])
def test_invalid_pitch_rejected(pitch):
    with pytest.raises(ValueError):
        camera_geometry(URDF, pitch)
