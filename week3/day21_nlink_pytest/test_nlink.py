"""test_nlink.py: pytest unit tests for nlink.py. Run with:  pytest -v"""

import numpy as np
import pytest

from nlink import fk_nlink, max_reach, tip_closed_form

L1, L2 = 2.0, 1.5


def tip(angles_deg, lengths):
    return fk_nlink(np.radians(angles_deg), lengths)[-1]


# ---- Cases you can check by eye (parametrize runs one test per row) ----
@pytest.mark.parametrize(
    "angles_deg, expected",
    [
        ((0, 0), (3.5, 0.0)),        # stretched along x
        ((90, 0), (0.0, 3.5)),       # stretched upward
        ((180, 0), (-3.5, 0.0)),     # stretched left
        ((0, 90), (2.0, 1.5)),       # forearm up
        ((90, -90), (1.5, 2.0)),     # upper arm up, forearm right
        ((0, 180), (0.5, 0.0)),      # folded back
        ((30, 60), (1.7321, 2.5)),   # Day 19 worked example
    ],
)
def test_two_link_known_poses(angles_deg, expected):
    assert tip(angles_deg, [L1, L2]) == pytest.approx(expected, abs=1e-3)


def test_three_link_by_hand():
    # lengths 1,1,1 with angles 90, -90, 90 -> absolute angles 90, 0, 90
    # positions: (0,1), (1,1), (1,2)
    pts = fk_nlink(np.radians([90, -90, 90]), [1, 1, 1])
    assert pts == pytest.approx(np.array([[0, 0], [0, 1], [1, 1], [1, 2]]), abs=1e-9)


def test_one_link_is_a_single_arrow():
    assert tip([60], [2.0]) == pytest.approx((1.0, 1.7321), abs=1e-3)


# ---- Structure of the result ----
def test_returns_base_and_every_joint():
    pts = fk_nlink(np.radians([10, 20, 30, 40]), [1, 1, 1, 1])
    assert pts.shape == (5, 2)          # base + 4 link ends
    assert pts[0] == pytest.approx((0, 0))


def test_link_lengths_are_preserved():
    rng = np.random.default_rng(1)
    lengths = [2.0, 1.5, 1.0, 0.5]
    pts = fk_nlink(rng.uniform(-np.pi, np.pi, 4), lengths)
    for i, length in enumerate(lengths):
        assert np.linalg.norm(pts[i + 1] - pts[i]) == pytest.approx(length)


# ---- Two independent methods must agree ----
def test_loop_matches_closed_form_on_random_arms():
    rng = np.random.default_rng(0)
    for _ in range(500):
        n = rng.integers(1, 8)
        angles = rng.uniform(-np.pi, np.pi, n)
        lengths = rng.uniform(0.2, 3.0, n)
        assert fk_nlink(angles, lengths)[-1] == pytest.approx(
            tip_closed_form(angles, lengths), abs=1e-9)


# ---- Physical limits ----
def test_tip_never_beyond_max_reach():
    rng = np.random.default_rng(2)
    lengths = [2.0, 1.5, 1.0]
    for _ in range(500):
        t = fk_nlink(rng.uniform(-np.pi, np.pi, 3), lengths)[-1]
        assert np.linalg.norm(t) <= max_reach(lengths) + 1e-9


def test_rotating_first_joint_rotates_whole_arm():
    lengths = [2.0, 1.5]
    base_tip = tip((0, 60), lengths)
    turned_tip = tip((90, 60), lengths)  # same shape, turned 90 deg about the base
    expected = np.array([-base_tip[1], base_tip[0]])
    assert turned_tip == pytest.approx(expected, abs=1e-9)


# ---- Bad input must be rejected ----
def test_mismatched_inputs_raise():
    with pytest.raises(ValueError):
        fk_nlink(np.radians([10, 20]), [1.0])


def test_empty_arm_raises():
    with pytest.raises(ValueError):
        fk_nlink([], [])


def test_negative_length_raises():
    with pytest.raises(ValueError):
        fk_nlink(np.radians([10]), [-1.0])
