import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1]))
import numpy as np
import pytest
from types import SimpleNamespace
from approach_object.model_io import image_array
Image = SimpleNamespace

def test_depth_endianness_and_row_padding():
    m = Image()
    m.encoding, m.height, m.width, m.step, m.is_bigendian = '32FC1', 2, 2, 12, True
    data = np.array([[100,200,0],[300,400,0]], dtype='>f4')
    m.data = data.tobytes()
    assert np.array_equal(image_array(m, '32FC1', 'float32'), [[100,200],[300,400]])

def test_bad_stride_fails_closed():
    m = Image()
    m.encoding, m.height, m.width, m.step = 'bgr8', 2, 2, 2
    m.data = bytes(4)
    with pytest.raises(ValueError): image_array(m, 'bgr8', 'uint8', 3)
