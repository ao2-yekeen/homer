"""Factory control scaling, invalid values, and absence of qualification claims."""
from pathlib import Path
import sys
from types import SimpleNamespace
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tof_camera_metadata import read_camera_metadata


def camera(values):
    return SimpleNamespace(getCameraInfo=lambda: SimpleNamespace(width=240, height=180),
                           getControl=lambda key: values[key])


CONTROLS = SimpleNamespace(INTRINSIC_FX='fx', INTRINSIC_FY='fy',
                           INTRINSIC_CX='cx', INTRINSIC_CY='cy', RANGE='range')


def test_factory_controls_are_scaled_once_and_remain_unqualified():
    result = read_camera_metadata(camera(dict(fx=21542, fy=21250, cx=11950, cy=8950, range=4000)), CONTROLS)
    assert result['K'] == [[215.42, 0., 119.5], [0., 212.5, 89.5], [0., 0., 1.]]
    assert result['factory_intrinsics_available'] is True
    assert result['physical_accuracy_verified'] is False
    assert result['depth_semantics'] == 'unvalidated' and result['distortion'] is None


@pytest.mark.parametrize('key,value', [('fx', 0), ('fy', -1), ('cx', 24000), ('cy', 18000)])
def test_invalid_sdk_control_values_rejected(key, value):
    values = dict(fx=21542, fy=21250, cx=11950, cy=8950, range=4000)
    values[key] = value
    with pytest.raises(ValueError, match='missing or invalid'):
        read_camera_metadata(camera(values), CONTROLS)
