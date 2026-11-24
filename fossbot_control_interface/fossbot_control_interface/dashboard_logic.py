"""Pure data handling for the dashboard (no ROS or GUI imports, easy to unit test)."""

from collections import deque
import math


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return the yaw angle [rad, -pi..pi] of a quaternion (rotation about the Z axis)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class TimeSeries:
    """Samples (t, value_1, ..., value_n) kept only for the last `history` seconds."""

    def __init__(self, history: float, n_values: int = 1):
        if history <= 0:
            raise ValueError('history must be positive')
        self.history = history
        self.n_values = n_values
        self._samples = deque()

    def append(self, t: float, *values: float):
        if len(values) != self.n_values:
            raise ValueError(f'expected {self.n_values} values, got {len(values)}')
        self._samples.append((t, *values))
        while self._samples and self._samples[0][0] < t - self.history:
            self._samples.popleft()

    def clear(self):
        self._samples.clear()

    def __len__(self):
        return len(self._samples)

    def columns(self, now: float = None):
        """
        Return ([t - now ...], [value_1 ...], ...): times relative to `now` (<= 0).

        `now` defaults to the newest sample. Samples older than `history` are dropped.
        """
        if not self._samples:
            return tuple([] for _ in range(self.n_values + 1))
        if now is None:
            now = self._samples[-1][0]
        rows = [s for s in self._samples if s[0] >= now - self.history]
        cols = list(zip(*rows)) if rows else [()] * (self.n_values + 1)
        times = [t - now for t in cols[0]]
        return (times, *(list(c) for c in cols[1:]))


class Trajectory:
    """XY path of the robot; a point is stored only after moving at least `min_step` metres."""

    def __init__(self, min_step: float = 0.01, max_points: int = 5000):
        if min_step < 0:
            raise ValueError('min_step must be >= 0')
        if max_points < 2:
            raise ValueError('max_points must be >= 2')
        self.min_step = min_step
        self.xs = deque(maxlen=max_points)
        self.ys = deque(maxlen=max_points)
        self.length = 0.0  # total distance travelled [m], not reduced when old points drop out

    def add(self, x: float, y: float) -> bool:
        """Add a pose; returns True if it was stored."""
        if self.xs:
            step = math.hypot(x - self.xs[-1], y - self.ys[-1])
            if step < self.min_step:
                return False
            self.length += step
        self.xs.append(x)
        self.ys.append(y)
        return True

    def clear(self):
        self.xs.clear()
        self.ys.clear()
        self.length = 0.0

    def __len__(self):
        return len(self.xs)


def square_limits(xs, ys, margin: float = 0.3, min_size: float = 1.0):
    """
    Return (x_min, x_max, y_min, y_max): a square window around all points.

    A square keeps the aspect ratio 1:1 so the drawn path is not distorted.
    """
    if not xs:
        half = min_size / 2.0
        return -half, half, -half, half
    x_lo, x_hi, y_lo, y_hi = min(xs), max(xs), min(ys), max(ys)
    size = max(x_hi - x_lo, y_hi - y_lo) + 2.0 * margin
    size = max(size, min_size)
    cx, cy = (x_lo + x_hi) / 2.0, (y_lo + y_hi) / 2.0
    return cx - size / 2.0, cx + size / 2.0, cy - size / 2.0, cy + size / 2.0


def update_square_limits(limits, xs, ys, margin: float = 0.3, min_size: float = 1.0):
    """
    Keep `limits` while all points stay at least margin/2 inside them, else recompute.

    Changing axis limits forces a full (slow) redraw of the plot, so the view
    only jumps when the robot gets close to its edge.
    """
    if limits is not None and xs:
        x_lo, x_hi, y_lo, y_hi = limits
        inner = margin / 2.0
        if (min(xs) >= x_lo + inner and max(xs) <= x_hi - inner
                and min(ys) >= y_lo + inner and max(ys) <= y_hi - inner):
            return limits
    if limits is not None and not xs:
        return limits
    return square_limits(xs, ys, margin, min_size)


def nice_ceil(value: float, minimum: float = 0.05) -> float:
    """
    Return the smallest number of the form {1, 2, 5} * 10^k that is >= value (and >= minimum).

    Used for the velocity axes, so their range changes in a few discrete steps
    instead of on every frame.
    """
    value = max(value, minimum)
    exponent = math.floor(math.log10(value))
    for exp in (exponent, exponent + 1):
        for mantissa in (1.0, 2.0, 5.0):
            candidate = mantissa * 10.0 ** exp
            if candidate >= value * (1 - 1e-9):
                return candidate
    return 10.0 ** (exponent + 1)  # not reached
