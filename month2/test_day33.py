import numpy as np
import pytest

from day33_movej_vs_movel import (ARM, END, START, distance_to_segment, movej_angles,
                                  movel_angles, peak_joint_speed_deg_s)


def test_both_moves_hit_start_and_end():
    aj = movej_angles(ARM, START, END)
    al, _ = movel_angles(ARM, START, END)
    for ang in (aj, al):
        s = ARM.tip_positions(ang[:1, 0], ang[:1, 1])[0]
        e = ARM.tip_positions(ang[-1:, 0], ang[-1:, 1])[0]
        assert s == pytest.approx(START, abs=1e-9)
        assert e == pytest.approx(END, abs=1e-9)


def test_movel_is_straight():
    al, _ = movel_angles(ARM, START, END)
    tips = ARM.tip_positions(al[:, 0], al[:, 1])
    assert distance_to_segment(tips, START, END).max() < 1e-9


def test_movej_is_not_straight():
    aj = movej_angles(ARM, START, END)
    tips = ARM.tip_positions(aj[:, 0], aj[:, 1])
    assert distance_to_segment(tips, START, END).max() > 0.01


def test_movej_joints_are_linear():
    aj = movej_angles(ARM, START, END)
    steps = np.diff(aj, axis=0)
    assert np.allclose(steps, steps[0])


def test_distance_to_segment_basic():
    pts = np.array([[0.5, 1.0], [2.0, 0.0], [-1.0, 0.0]])
    d = distance_to_segment(pts, (0, 0), (1, 0))
    assert d == pytest.approx([1.0, 1.0, 1.0])


def test_peak_speed():
    ang = np.radians(np.array([[0.0, 0.0], [10.0, 0.0], [20.0, 0.0]]))
    assert peak_joint_speed_deg_s(ang, 2.0) == pytest.approx(10.0)