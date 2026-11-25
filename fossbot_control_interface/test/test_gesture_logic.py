import math

from fossbot_control_interface.gesture_logic import (
    classify, Gesture, GestureDebouncer, rotation_speed, RotationTracker, wrap_angle)
import pytest

# finger name -> (PIP index, TIP index); the remaining landmarks stay near the wrist
FINGER_IDS = {'index': (6, 8), 'middle': (10, 12), 'ring': (14, 16), 'pinky': (18, 20)}


def make_hand(extended, rotation=0.0, scale=100.0, origin=(320.0, 400.0)):
    """Build 21 (x, y) landmarks; `extended` is the set of extended fingers."""
    points = [origin] * 21
    for i, (name, (pip, tip)) in enumerate(FINGER_IDS.items()):
        # fingers fan out around "up" in the image (-y), then the hand is rotated
        angle = -math.pi / 2 + (i - 1.5) * 0.25 + rotation
        tip_dist = 2.0 if name in extended else 0.8   # folded: tip bends back to the palm
        for idx, dist in ((pip, 1.0), (tip, tip_dist)):
            points[idx] = (origin[0] + scale * dist * math.cos(angle),
                           origin[1] + scale * dist * math.sin(angle))
    return points


@pytest.mark.parametrize('rotation', [0.0, 0.8, -1.2, math.pi])
def test_gestures_any_hand_rotation(rotation):
    assert classify(make_hand({'index', 'middle', 'ring'}, rotation)) == Gesture.THREE
    assert classify(make_hand({'index', 'pinky'}, rotation)) == Gesture.HORNS
    assert classify(make_hand({'index', 'middle', 'ring', 'pinky'}, rotation)) == Gesture.PALM


@pytest.mark.parametrize('extended', [
    set(),                         # fist (Lab 08: stop)
    {'index'},                     # Lab 08: forward
    {'index', 'middle'},           # victory: two fingers are not THREE
    {'middle', 'ring', 'pinky'},
    {'middle'},
    {'pinky'},
])
def test_other_poses_are_none(extended):
    assert classify(make_hand(extended)) == Gesture.NONE


def test_no_hand_is_none():
    assert classify(None) == Gesture.NONE
    assert classify([(0.0, 0.0)] * 5) == Gesture.NONE


def test_debouncer_fires_once_after_hold():
    d = GestureDebouncer(hold_frames=3)
    events = [d.update(Gesture.THREE) for _ in range(10)]
    assert events == [None, None, Gesture.THREE] + [None] * 7


def test_debouncer_needs_release_and_resets_on_flicker():
    d = GestureDebouncer(hold_frames=2)
    assert d.update(Gesture.HORNS) is None
    assert d.update(Gesture.NONE) is None        # flicker resets the count
    assert d.update(Gesture.HORNS) is None
    assert d.update(Gesture.HORNS) == Gesture.HORNS
    assert d.update(Gesture.HORNS) is None       # still held: no second trigger
    d.update(Gesture.NONE)
    d.update(Gesture.HORNS)
    assert d.update(Gesture.HORNS) == Gesture.HORNS


def test_debouncer_progress():
    d = GestureDebouncer(hold_frames=4)
    assert d.progress == 0.0
    d.update(Gesture.THREE)
    assert d.progress == pytest.approx(0.25)
    for _ in range(3):
        d.update(Gesture.THREE)
    assert d.progress == 0.0   # fired


def test_debouncer_rejects_bad_hold():
    with pytest.raises(ValueError):
        GestureDebouncer(0)


def test_wrap_angle():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert wrap_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)


def simulate_yaws(start, step, n):
    """Yaw readings as /odom gives them: wrapped to [-pi, pi)."""
    return [wrap_angle(start + step * i) for i in range(n)]


@pytest.mark.parametrize('direction', [1, -1])
@pytest.mark.parametrize('start', [0.0, 3.0, -3.0])
def test_tracker_counts_full_turn_across_wrap(direction, start):
    tracker = RotationTracker(direction)
    step = direction * 0.05
    readings = simulate_yaws(start, step, int(2 * math.pi / 0.05) + 1)
    for yaw in readings:
        tracker.update(yaw)
    assert tracker.turned == pytest.approx(0.05 * (len(readings) - 1))
    assert tracker.remaining == pytest.approx(2 * math.pi - tracker.turned)


def test_tracker_wrong_direction_is_negative_progress():
    tracker = RotationTracker(+1)
    for yaw in simulate_yaws(0.0, -0.1, 5):
        tracker.update(yaw)
    assert tracker.turned == pytest.approx(-0.4)
    assert tracker.remaining == pytest.approx(2 * math.pi + 0.4)


def test_tracker_first_reading_only_sets_reference():
    tracker = RotationTracker(-1)
    assert not tracker.started
    assert tracker.update(1.0) == pytest.approx(2 * math.pi)
    assert tracker.started


def test_tracker_rejects_bad_direction():
    with pytest.raises(ValueError):
        RotationTracker(0)


def test_rotation_speed_profile():
    assert rotation_speed(3.0, 0.8, 0.2, 0.6) == pytest.approx(0.8)
    assert rotation_speed(0.3, 0.8, 0.2, 0.6) == pytest.approx(0.5)
    assert rotation_speed(1e-6, 0.8, 0.2, 0.6) == pytest.approx(0.2, abs=1e-4)
    assert rotation_speed(0.0, 0.8, 0.2, 0.6) == 0.0
    assert rotation_speed(-0.1, 0.8, 0.2, 0.6) == 0.0
    assert rotation_speed(0.1, 0.8, 0.2, 0.0) == pytest.approx(0.8)


def test_rotation_speed_rejects_bad_limits():
    with pytest.raises(ValueError):
        rotation_speed(1.0, 0.2, 0.8, 0.5)
    with pytest.raises(ValueError):
        rotation_speed(1.0, 0.8, 0.0, 0.5)
