"""Unit conversion helpers."""

import numpy as np


def mm_to_m(
    point_mm: tuple[float, float, float] | np.ndarray,
) -> list[float]:
    """Convert a point or vector from millimeters to meters."""
    return [value / 1000.0 for value in point_mm]


def m_to_mm(
    point_m: tuple[float, float, float] | np.ndarray,
) -> list[float]:
    """Convert a point or vector from meters to millimeters."""
    return [value * 1000.0 for value in point_m]
