"""test_ik.py: pytest tests for ik.py. Run with:  python -m pytest -v"""

import math

import numpy as np
import pytest

from ik import (UnreachableError, fk_2link, ik_2link, is_reachable,
                nearest_reachable, reach_limits)

L1, L2 = 2.0, 1.5


# ---- IK answers must reproduce the target through FK ----
@pytest.mark.parametrize("target", [(1.8, 2.2), (-1.0, 1.5), (0.0, 2.0),
                                    (-2.0, -1.0), (2.5, -0.5)])
@pytest.mark.parametrize("elbow_up", [False, True])
def test_ik_then_fk_returns_target(target, elbow_up):
    t1, t2 = ik_2link(*target, L1, L2, elbow_up=elbow_up)
    _, tip = fk_2link(t1, t2, L1, L2)
    assert tip == pytest.approx(target, abs=1e-9)


def test_known_answer_from_day_22():
    t1, t2 = ik_2link(1.8, 2.2, L1, L2, elbow_up=False)
    assert math.degrees(t1) == pytest.approx(20.54, abs=0.01)
    assert math.degrees(t2) == pytest.approx(72.24, abs=0.01)


def test_elbow_flag_flips_sign_of_theta2():
    _, down = ik_2link(1.8, 2.2, L1, L2, elbow_up=False)
    _, up = ik_2link(1.8, 2.2, L1, L2, elbow_up=True)
    assert down > 0 > up
    assert down == pytest.approx(-up)


# ---- Reach limits ----
def test_limits():
    assert reach_limits(L1, L2) == (0.5, 3.5)


def test_exact_max_reach_works():
    t1, t2 = ik_2link(3.5, 0.0, L1, L2)
    assert t1 == pytest.approx(0.0)
    assert t2 == pytest.approx(0.0)


def test_exact_min_reach_works():
    _, t2 = ik_2link(0.5, 0.0, L1, L2)
    assert abs(t2) == pytest.approx(math.pi)


def test_too_far_raises_with_clear_message():
    with pytest.raises(UnreachableError, match="out of reach"):
        ik_2link(4.0, 0.0, L1, L2)


def test_dead_zone_raises():
    with pytest.raises(UnreachableError, match="dead zone"):
        ik_2link(0.1, 0.1, L1, L2)


def test_unreachable_error_is_a_value_error():
    with pytest.raises(ValueError):
        ik_2link(10.0, 10.0, L1, L2)


def test_is_reachable():
    assert is_reachable(1.8, 2.2, L1, L2)
    assert not is_reachable(4.0, 0.0, L1, L2)
    assert not is_reachable(0.0, 0.0, L1, L2)


# ---- Special cases and bad input ----
def test_equal_links_target_at_base_is_allowed():
    t1, t2 = ik_2link(0.0, 0.0, 1.0, 1.0)
    _, tip = fk_2link(t1, t2, 1.0, 1.0)
    assert tip == pytest.approx((0.0, 0.0), abs=1e-9)


def test_unequal_links_target_at_base_is_unreachable():
    with pytest.raises(UnreachableError):
        ik_2link(0.0, 0.0, 2.0, 1.0)


def test_bad_link_length_raises():
    with pytest.raises(ValueError):
        ik_2link(1.0, 1.0, 0.0, 1.0)


def test_nan_target_raises():
    with pytest.raises(ValueError):
        ik_2link(float("nan"), 1.0, L1, L2)


# ---- Random targets: every reachable one must round-trip ----
def test_random_reachable_targets_round_trip():
    rng = np.random.default_rng(0)
    r_min, r_max = reach_limits(L1, L2)
    for _ in range(1000):
        r = rng.uniform(r_min, r_max)
        a = rng.uniform(-math.pi, math.pi)
        target = (r * math.cos(a), r * math.sin(a))
        for elbow_up in (False, True):
            t1, t2 = ik_2link(*target, L1, L2, elbow_up=elbow_up)
            _, tip = fk_2link(t1, t2, L1, L2)
            assert tip == pytest.approx(target, abs=1e-8)


# ---- nearest_reachable ----
def test_nearest_reachable_clamps_far_target():
    x, y = nearest_reachable(7.0, 0.0, L1, L2)
    assert (x, y) == pytest.approx((3.5, 0.0))


def test_nearest_reachable_leaves_good_target_alone():
    assert nearest_reachable(1.8, 2.2, L1, L2) == pytest.approx((1.8, 2.2))


def test_nearest_reachable_pushes_out_of_dead_zone():
    x, y = nearest_reachable(0.1, 0.0, L1, L2)
    assert math.hypot(x, y) == pytest.approx(0.5)
