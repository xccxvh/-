# -*- coding: utf-8 -*-
"""Bump when the definition of the measured pixel coordinate changes."""
COORDINATE_VERSION = 'geometry-envelope-v2'


def calibrated_method(method):
    """Only the original envelope estimator has verified cm calibration.

    Hysteresis recovery changes the brightness contour and must be validated
    independently before its pixel coordinate may use the old cm mapping.
    """
    return method in ('geom', 'hybrid')

# 原图球冠定义与轨道轴坐标各自版本化，不改变历史geometry-envelope-v2合同。
RELATIVE_CAP_VERSION = 'relative-cap-v1'
RAIL_COORDINATE_VERSION = 'rail-reference-v1'
IMAGE_SPACES = ('raw_image', 'undistorted_image')
