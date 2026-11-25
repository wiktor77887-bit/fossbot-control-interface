"""
Pure logic for gesture control (no ROS or MediaPipe imports, easy to unit test).

Gestures are recognised from the 21 MediaPipe hand landmarks. They are
deliberately different from the Lab 08 set (index = forward, open hand = back,
thumb to the side = left/right, anything else = stop):

* THREE - index, middle and ring finger extended, pinky folded
* HORNS - index and pinky extended, middle and ring folded
* PALM  - all four fingers extended (used only to abort a rotation)

The thumb is ignored: people hold it differently in these poses and it is the
least reliable landmark.
"""

from enum import Enum
import math

# MediaPipe hand landmark indices
WRIST = 0
INDEX_PIP, INDEX_TIP = 6, 8
MIDDLE_PIP, MIDDLE_TIP = 10, 12
RING_PIP, RING_TIP = 14, 16
PINKY_PIP, PINKY_TIP = 18, 20

FINGERS = (
    (INDEX_PIP, INDEX_TIP),
    (MIDDLE_PIP, MIDDLE_TIP),
    (RING_PIP, RING_TIP),
    (PINKY_PIP, PINKY_TIP),
)


class Gesture(Enum):
    """Recognised hand poses."""

    NONE = 'none'
    THREE = 'three'
    HORNS = 'horns'
    PALM = 'palm'


def _dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def fingers_extended(points, ratio: float = 1.2):
    """
    Return (index, middle, ring, pinky) flags: True if the finger is extended.

    A finger counts as extended when its tip is clearly farther from the wrist
    than its PIP joint. Comparing distances from the wrist (instead of "tip
    above joint", as in Lab 08) works for any hand rotation in the image.
    `points` are (x, y) landmarks in pixels, so the image aspect ratio does not
    distort the distances.
    """
    wrist = points[WRIST]
    return tuple(
        _dist(points[tip], wrist) > ratio * _dist(points[pip], wrist)
        for pip, tip in FINGERS)


def classify(points, ratio: float = 1.2) -> Gesture:
    """Classify 21 (x, y) landmarks as one of the gestures."""
    if points is None or len(points) < 21:
        return Gesture.NONE
    pattern = fingers_extended(points, ratio)
    if pattern == (True, True, True, False):
        return Gesture.THREE
    if pattern == (True, False, False, True):
        return Gesture.HORNS
    if pattern == (True, True, True, True):
        return Gesture.PALM
    return Gesture.NONE


class GestureDebouncer:
    """
    Turn per-frame classifications into single trigger events.

    A gesture fires once after it was seen in `hold_frames` consecutive frames.
    It fires again only after the hand showed something else in between, so
    holding the pose does not start rotation after rotation.
    """

    def __init__(self, hold_frames: int = 5):
        if hold_frames < 1:
            raise ValueError('hold_frames must be >= 1')
        self.hold_frames = hold_frames
        self._current = Gesture.NONE
        self._count = 0
        self._fired = False

    def update(self, gesture: Gesture):
        """Feed one frame; return the gesture when it fires, otherwise None."""
        if gesture != self._current:
            self._current = gesture
            self._count = 0
            self._fired = False
        self._count += 1
        if (gesture != Gesture.NONE and not self._fired
                and self._count >= self.hold_frames):
            self._fired = True
            return gesture
        return None

    @property
    def progress(self) -> float:
        """Return how far the current gesture is towards firing (0..1)."""
        if self._current == Gesture.NONE or self._fired:
            return 0.0
        return min(1.0, self._count / self.hold_frames)


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class RotationTracker:
    """
    Closed-loop progress of an in-place rotation measured from odometry yaw.

    Yaw from /odom jumps between +pi and -pi, so the rotation is summed from
    small wrapped differences between consecutive readings. `direction` is +1
    for counter-clockwise (positive angular.z) and -1 for clockwise.
    """

    def __init__(self, direction: int, target: float = 2.0 * math.pi):
        if direction not in (1, -1):
            raise ValueError('direction must be +1 or -1')
        self.direction = direction
        self.target = target
        self.turned = 0.0          # [rad] progress in the commanded direction
        self._last_yaw = None

    def update(self, yaw: float) -> float:
        """Add a new yaw reading; return the angle still to turn [rad]."""
        if self._last_yaw is not None:
            self.turned += self.direction * wrap_angle(yaw - self._last_yaw)
        self._last_yaw = yaw
        return self.remaining

    @property
    def remaining(self) -> float:
        """Return the angle still to turn [rad] (negative after an overshoot)."""
        return self.target - self.turned

    @property
    def started(self) -> bool:
        """Return True once the first yaw reading arrived."""
        return self._last_yaw is not None


def rotation_speed(remaining: float, max_speed: float, min_speed: float,
                   slow_zone: float) -> float:
    """
    Return the angular speed magnitude [rad/s] for the angle still to turn.

    Full speed until the last `slow_zone` radians, then a linear ramp down to
    `min_speed`, so the robot does not overshoot because of odometry delay.
    Returns 0 when nothing is left.
    """
    if not 0.0 < min_speed <= max_speed:
        raise ValueError('expected 0 < min_speed <= max_speed')
    if remaining <= 0.0:
        return 0.0
    if slow_zone <= 0.0 or remaining >= slow_zone:
        return max_speed
    return min_speed + (max_speed - min_speed) * remaining / slow_zone
