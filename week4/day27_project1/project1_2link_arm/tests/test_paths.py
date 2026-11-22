import numpy as np
import pytest

from arm2link import (TwoLinkArm, UnreachableError, circle_path, joint_steps_deg,
                      line_path, solve_path, tracking_error)

ARM = TwoLinkArm(2.0, 1.5)


def test_circle_is_closed():
    p = circle_path((1.5, 1.0), 1.0, 100)
    assert p.shape == (101, 2)
    assert p[0] == pytest.approx(p[-1])


def test_line_endpoints():
    p = line_path((-2, 1), (2, 2), 50)
    assert p[0] == pytest.approx([-2, 1])
    assert p[-1] == pytest.approx([2, 2])


@pytest.mark.parametrize("elbow_up", [False, True])
def test_circle_tracking_error_tiny(elbow_up):
    p = circle_path((1.5, 1.0), 1.0, 120)
    a = solve_path(ARM, p, elbow_up=elbow_up)
    assert tracking_error(ARM, p, a).max() < 1e-9


def test_line_tracking_error_tiny():
    p = line_path((-2, 1), (2, 2), 120)
    a = solve_path(ARM, p)
    assert tracking_error(ARM, p, a).max() < 1e-9


def test_line_through_dead_zone_names_waypoint():
    p = line_path((-2, 0), (2, 0), 120)
    with pytest.raises(UnreachableError, match="Waypoint 45"):
        solve_path(ARM, p)


def test_circle_too_big_fails_at_waypoint_0():
    p = circle_path((1.5, 1.0), 2.5, 60)
    with pytest.raises(UnreachableError, match="Waypoint 0"):
        solve_path(ARM, p)


def test_no_big_jumps_after_unwrap():
    p = circle_path((1.5, 1.0), 1.0, 120)
    a = solve_path(ARM, p)
    assert joint_steps_deg(a).max() < 10


def test_joint_steps_shape():
    a = np.zeros((5, 2))
    assert joint_steps_deg(a).shape == (4, 2)
