import pytest

from fossbot_control_interface.click_logic import click_to_linear_velocity

H = 480


def test_top_edge_is_full_forward():
    assert click_to_linear_velocity(0, H, max_speed=0.2) == pytest.approx(0.2)


def test_bottom_edge_is_full_backward():
    assert click_to_linear_velocity(H, H, max_speed=0.2) == pytest.approx(-0.2)


def test_centre_is_stop():
    assert click_to_linear_velocity(H / 2, H, max_speed=0.2) == 0.0


def test_dead_zone_is_stop():
    # 5 % of half-height = 12 px around the centre
    assert click_to_linear_velocity(H / 2 - 10, H, 0.2, dead_zone=0.05) == 0.0
    assert click_to_linear_velocity(H / 2 + 10, H, 0.2, dead_zone=0.05) == 0.0


def test_min_speed_just_outside_dead_zone():
    v = click_to_linear_velocity(H / 2 - 13, H, 0.2, min_speed=0.05, dead_zone=0.05)
    assert 0.05 <= v < 0.06


def test_speed_grows_with_distance():
    near = click_to_linear_velocity(200, H, 0.2, min_speed=0.05)
    far = click_to_linear_velocity(50, H, 0.2, min_speed=0.05)
    assert 0 < near < far


def test_non_proportional_always_max():
    assert click_to_linear_velocity(200, H, 0.2, proportional=False) == pytest.approx(0.2)
    assert click_to_linear_velocity(300, H, 0.2, proportional=False) == pytest.approx(-0.2)


def test_click_outside_window_is_clamped():
    assert click_to_linear_velocity(-100, H, 0.2) == pytest.approx(0.2)
    assert click_to_linear_velocity(H + 100, H, 0.2) == pytest.approx(-0.2)


@pytest.mark.parametrize('kwargs', [
    {'height': 0},
    {'dead_zone': 1.0},
    {'min_speed': 0.3},
])
def test_invalid_arguments(kwargs):
    args = {'y': 10, 'height': H, 'max_speed': 0.2}
    args.update(kwargs)
    with pytest.raises(ValueError):
        click_to_linear_velocity(**args)
