# -*- coding: utf-8 -*-
"""Bump when the definition of the measured pixel coordinate changes."""
COORDINATE_VERSION = 'geometry-envelope-v2'


def calibrated_method(method):
    """Only the original envelope estimator has verified cm calibration.

    Hysteresis recovery changes the brightness contour and must be validated
    independently before its pixel coordinate may use the old cm mapping.
    """
    return method in ('geom', 'hybrid')
