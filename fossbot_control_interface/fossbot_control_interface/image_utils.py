"""
Convert sensor_msgs/Image to an OpenCV BGR array without cv_bridge.

cv_bridge is compiled against the system NumPy, which often breaks after
`pip install mediapipe` pulls in a different NumPy version. Doing the
conversion by hand keeps the package independent of that problem.
"""

import cv2
import numpy as np

# encoding -> (channels, cv2 conversion code or None if already BGR)
_ENCODINGS = {
    'bgr8': (3, None),
    'rgb8': (3, cv2.COLOR_RGB2BGR),
    'bgra8': (4, cv2.COLOR_BGRA2BGR),
    'rgba8': (4, cv2.COLOR_RGBA2BGR),
    'mono8': (1, cv2.COLOR_GRAY2BGR),
    '8uc1': (1, cv2.COLOR_GRAY2BGR),
    '8uc3': (3, None),
    'yuv422_yuy2': (2, cv2.COLOR_YUV2BGR_YUYV),
    'yuyv': (2, cv2.COLOR_YUV2BGR_YUYV),
    'yuv422': (2, cv2.COLOR_YUV2BGR_UYVY),  # in ROS "yuv422" means UYVY ordering
    'uyvy': (2, cv2.COLOR_YUV2BGR_UYVY),
}


def supported_encodings():
    return sorted(_ENCODINGS)


def image_to_bgr(msg) -> np.ndarray:
    """Return a (height, width, 3) uint8 BGR image built from a sensor_msgs/Image."""
    encoding = msg.encoding.lower()
    if encoding not in _ENCODINGS:
        raise ValueError(
            f"Unsupported image encoding '{msg.encoding}'. "
            f"Supported: {', '.join(supported_encodings())}")
    channels, conversion = _ENCODINGS[encoding]

    try:
        buf = np.frombuffer(msg.data, dtype=np.uint8)
    except TypeError:
        buf = np.asarray(msg.data, dtype=np.uint8)

    height, width, step = msg.height, msg.width, msg.step
    row_bytes = width * channels
    if step < row_bytes or buf.size < height * step:
        raise ValueError('Image data is smaller than width/height/step describe')

    # rows may be padded (step > width * channels), so cut each row first
    image = buf[:height * step].reshape(height, step)[:, :row_bytes]
    if channels > 1:
        image = image.reshape(height, width, channels)
    else:
        image = image.reshape(height, width)

    if conversion is None:
        return np.ascontiguousarray(image)
    return cv2.cvtColor(np.ascontiguousarray(image), conversion)
