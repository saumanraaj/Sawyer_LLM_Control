"""Utility functions for the vision tracker."""

from __future__ import annotations

import math


def euclidean_distance(p1: tuple[float, float], p2: tuple[float, float]) -> float:
    """Euclidean distance between two 2D points."""
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def ema_smooth(current: float, previous: float, alpha: float) -> float:
    """Exponential moving average: alpha * current + (1 - alpha) * previous."""
    return alpha * current + (1.0 - alpha) * previous
