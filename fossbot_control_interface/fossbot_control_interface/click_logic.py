"""Pure logic for the click-based teleoperation (no ROS imports, easy to unit test)."""

import math


def click_to_linear_velocity(
    y: float,
    height: float,
    max_speed: float,
    min_speed: float = 0.0,
    dead_zone: float = 0.05,
    proportional: bool = True,
) -> float:
    """
    Map the vertical position of a click to a forward/backward velocity.

    Parameters
    ----------
    y
        Vertical pixel coordinate of the click (0 = top edge of the window).
    height
        Window height in pixels.
    max_speed
        Speed [m/s] used at the top/bottom edge (or always, if not proportional).
    min_speed
        Speed [m/s] used just outside the dead zone (proportional mode only).
    dead_zone
        Fraction of half-height around the centre line where clicks are ignored.
    proportional
        If True, speed grows with the distance from the centre line.

    Returns
    -------
    float
        Linear velocity [m/s]: positive above the centre (forward), negative below
        (backward), 0.0 inside the dead zone.

    """
    if height <= 0:
        raise ValueError('height must be positive')
    if not 0.0 <= dead_zone < 1.0:
        raise ValueError('dead_zone must be in [0, 1)')
    if min_speed < 0 or max_speed < 0 or min_speed > max_speed:
        raise ValueError('expected 0 <= min_speed <= max_speed')

    centre = height / 2.0
    # +1 at the top edge, -1 at the bottom edge
    offset = max(-1.0, min(1.0, (centre - y) / centre))

    if abs(offset) <= dead_zone:
        return 0.0
    if not proportional:
        return math.copysign(max_speed, offset)

    ratio = (abs(offset) - dead_zone) / (1.0 - dead_zone)
    speed = min_speed + (max_speed - min_speed) * ratio
    return math.copysign(speed, offset)
