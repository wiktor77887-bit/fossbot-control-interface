import math

from fossbot_control_interface.dashboard_logic import (
    nice_ceil, quaternion_to_yaw, square_limits, TimeSeries, Trajectory, update_square_limits)
import pytest


def yaw_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


@pytest.mark.parametrize('yaw', [0.0, 0.5, math.pi / 2, -2.0, 3.0])
def test_quaternion_to_yaw(yaw):
    assert quaternion_to_yaw(*yaw_quaternion(yaw)) == pytest.approx(yaw)


def test_identity_quaternion_is_zero_yaw():
    assert quaternion_to_yaw(0.0, 0.0, 0.0, 1.0) == 0.0


def test_time_series_drops_old_samples():
    ts = TimeSeries(history=2.0)
    for t in range(6):
        ts.append(float(t), t * 10.0)
    times, values = ts.columns()
    assert times == [-2.0, -1.0, 0.0]
    assert values == [30.0, 40.0, 50.0]


def test_time_series_relative_to_now():
    ts = TimeSeries(history=5.0, n_values=2)
    ts.append(1.0, 0.1, 0.2)
    ts.append(2.0, 0.3, 0.4)
    times, v, w = ts.columns(now=4.0)
    assert times == [-3.0, -2.0]
    assert v == [0.1, 0.3] and w == [0.2, 0.4]


def test_time_series_hides_samples_older_than_history_at_now():
    ts = TimeSeries(history=1.0)
    ts.append(0.0, 1.0)
    times, values = ts.columns(now=10.0)  # no new data for a long time
    assert times == [] and values == []


def test_time_series_empty_and_clear():
    ts = TimeSeries(history=1.0, n_values=2)
    assert ts.columns() == ([], [], [])
    ts.append(0.0, 1.0, 2.0)
    ts.clear()
    assert len(ts) == 0


def test_time_series_wrong_value_count():
    with pytest.raises(ValueError):
        TimeSeries(history=1.0, n_values=2).append(0.0, 1.0)


def test_trajectory_skips_tiny_moves_and_sums_length():
    tr = Trajectory(min_step=0.1)
    assert tr.add(0.0, 0.0)
    assert not tr.add(0.05, 0.0)    # too close, ignored
    assert tr.add(0.3, 0.4)         # 0.5 m from the first point
    assert len(tr) == 2
    assert tr.length == pytest.approx(0.5)


def test_trajectory_max_points_keeps_newest():
    tr = Trajectory(min_step=0.0, max_points=3)
    for i in range(5):
        tr.add(float(i), 0.0)
    assert list(tr.xs) == [2.0, 3.0, 4.0]
    assert tr.length == pytest.approx(4.0)  # distance is not lost with old points


def test_trajectory_clear():
    tr = Trajectory()
    tr.add(0.0, 0.0)
    tr.add(1.0, 0.0)
    tr.clear()
    assert len(tr) == 0 and tr.length == 0.0


def test_square_limits_empty_uses_min_size():
    assert square_limits([], [], min_size=2.0) == (-1.0, 1.0, -1.0, 1.0)


def test_square_limits_is_square_and_contains_points():
    xs, ys = [0.0, 4.0], [1.0, 2.0]
    x_lo, x_hi, y_lo, y_hi = square_limits(xs, ys, margin=0.5)
    assert x_hi - x_lo == pytest.approx(y_hi - y_lo) == pytest.approx(5.0)
    assert x_lo <= min(xs) and x_hi >= max(xs)
    assert y_lo <= min(ys) and y_hi >= max(ys)


@pytest.mark.parametrize('value, expected', [
    (0.0, 0.05), (0.05, 0.05), (0.06, 0.1), (0.15, 0.2), (0.2, 0.2),
    (0.21, 0.5), (0.7, 1.0), (3.0, 5.0), (12.0, 20.0),
])
def test_nice_ceil(value, expected):
    assert nice_ceil(value) == pytest.approx(expected)


def test_update_square_limits_keeps_view_while_inside():
    limits = square_limits([0.0], [0.0], margin=0.5, min_size=2.0)  # -1..1
    assert update_square_limits(limits, [0.0, 0.5], [0.0, -0.5], margin=0.5) is limits


def test_update_square_limits_grows_near_edge():
    limits = (-1.0, 1.0, -1.0, 1.0)
    new = update_square_limits(limits, [0.0, 0.9], [0.0, 0.0], margin=0.5)
    assert new != limits
    assert new[1] >= 0.9 + 0.25


def test_update_square_limits_first_call_and_empty():
    assert update_square_limits(None, [], [], min_size=2.0) == (-1.0, 1.0, -1.0, 1.0)
    limits = (0.0, 1.0, 0.0, 1.0)
    assert update_square_limits(limits, [], []) is limits
