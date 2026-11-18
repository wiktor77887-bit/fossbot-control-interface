from types import SimpleNamespace

import numpy as np
import pytest

from fossbot_control_interface.image_utils import image_to_bgr


def make_msg(array, encoding, step=None):
    h, w = array.shape[:2]
    channels = 1 if array.ndim == 2 else array.shape[2]
    step = step or w * channels
    rows = np.zeros((h, step), dtype=np.uint8)
    rows[:, :w * channels] = array.reshape(h, w * channels)
    return SimpleNamespace(height=h, width=w, step=step, encoding=encoding, data=rows.tobytes())


def test_bgr8_passthrough():
    img = np.random.randint(0, 255, (4, 6, 3), dtype=np.uint8)
    assert np.array_equal(image_to_bgr(make_msg(img, 'bgr8')), img)


def test_rgb8_is_swapped():
    rgb = np.zeros((2, 2, 3), dtype=np.uint8)
    rgb[..., 0] = 255  # pure red in RGB
    bgr = image_to_bgr(make_msg(rgb, 'rgb8'))
    assert (bgr[..., 2] == 255).all() and (bgr[..., 0] == 0).all()


def test_mono8_becomes_three_channels():
    gray = np.full((3, 5), 77, dtype=np.uint8)
    out = image_to_bgr(make_msg(gray, 'mono8'))
    assert out.shape == (3, 5, 3) and (out == 77).all()


def test_padded_rows_are_cut():
    img = np.random.randint(0, 255, (4, 6, 3), dtype=np.uint8)
    assert np.array_equal(image_to_bgr(make_msg(img, 'bgr8', step=32)), img)


def test_yuyv_shape():
    yuyv = np.full((4, 6, 2), 128, dtype=np.uint8)
    assert image_to_bgr(make_msg(yuyv, 'yuv422_yuy2')).shape == (4, 6, 3)


def test_unsupported_encoding():
    with pytest.raises(ValueError):
        image_to_bgr(make_msg(np.zeros((2, 2), dtype=np.uint8), '16UC1'))


def test_truncated_data():
    msg = make_msg(np.zeros((4, 4, 3), dtype=np.uint8), 'bgr8')
    msg.data = msg.data[:10]
    with pytest.raises(ValueError):
        image_to_bgr(msg)
