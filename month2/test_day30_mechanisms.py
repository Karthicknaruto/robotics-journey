import math

import pytest

from day30_mechanisms import (braced_triangle_dof, delta_robot_dof, four_bar_dof,
                              four_bar_rocker, serial_arm_dof)


def test_four_bar():
    assert four_bar_dof() == 1


@pytest.mark.parametrize("n", [1, 2, 3, 6, 7])
def test_serial_arm_dof_equals_joints(n):
    assert serial_arm_dof(n) == n


def test_delta():
    raw, idle, real = delta_robot_dof()
    assert (raw, idle, real) == (9, 6, 3)


def test_braced_triangle_negative():
    assert braced_triangle_dof() == -1


def test_four_bar_closes_everywhere():
    for deg in range(0, 360, 3):
        assert four_bar_rocker(math.radians(deg)) is not None


def test_four_bar_too_short_cannot_close():
    # Coupler and rocker too short to reach: the linkage cannot assemble
    assert four_bar_rocker(0.0, ground=10.0, crank=1.0, coupler=2.0, rocker=2.0) is None
    