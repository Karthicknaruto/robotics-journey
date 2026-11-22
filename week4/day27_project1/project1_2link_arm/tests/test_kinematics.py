import math

import numpy as np
import pytest

from arm2link import TwoLinkArm, UnreachableError

ARM = TwoLinkArm(2.0, 1.5)


def test_rejects_bad_lengths():
    with pytest.raises(ValueError):
        TwoLinkArm(0, 1)
    with pytest.raises(ValueError):
        TwoLinkArm(1, -2)


def test_reach_limits():
    assert ARM.max_reach == pytest.approx(3.5)
    assert ARM.min_reach == pytest.approx(0.5)


@pytest.mark.parametrize("x,y,expected", [
    (3.5, 0, True), (0.5, 0, True), (2, 2, True),
    (3.6, 0, False), (0.2, 0, False), (0, 0, False),
])
def test_is_reachable(x, y, expected):
    assert ARM.is_reachable(x, y) is expected


def test_fk_straight_arm():
    elbow, tip = ARM.fk(0, 0)
    assert elbow == pytest.approx([2.0, 0.0])
    assert tip == pytest.approx([3.5, 0.0])


def test_fk_right_angle():
    _, tip = ARM.fk(0, math.pi / 2)
    assert tip == pytest.approx([2.0, 1.5])


@pytest.mark.parametrize("t1,t2", [(0.3, 0.8), (-1.2, 2.0), (2.5, -1.0), (0, math.pi)])
def test_chain_matches_closed_form(t1, t2):
    _, tip = ARM.fk(t1, t2)
    assert tip == pytest.approx(ARM.tip_positions([t1], [t2])[0])


@pytest.mark.parametrize("elbow_up", [False, True])
@pytest.mark.parametrize("x,y", [(2, 2), (-1, 1.5), (0.6, -0.1), (3.5, 0), (0, -3)])
def test_ik_fk_round_trip(x, y, elbow_up):
    t1, t2 = ARM.ik(x, y, elbow_up=elbow_up)
    _, tip = ARM.fk(t1, t2)
    assert tip == pytest.approx([x, y], abs=1e-9)


def test_elbow_sign_convention():
    assert ARM.ik(2, 2, elbow_up=False)[1] > 0
    assert ARM.ik(2, 2, elbow_up=True)[1] < 0


def test_out_of_reach():
    with pytest.raises(UnreachableError, match="out of reach"):
        ARM.ik(4, 0)


def test_dead_zone():
    with pytest.raises(UnreachableError, match="dead zone"):
        ARM.ik(0.1, 0.1)


def test_unreachable_is_value_error():
    with pytest.raises(ValueError):
        ARM.ik(10, 10)


def test_non_finite():
    with pytest.raises(ValueError):
        ARM.ik(float("nan"), 1)


def test_exact_limit_is_accepted():
    t1, t2 = ARM.ik(3.5, 0)
    assert t2 == pytest.approx(0, abs=1e-6)


def test_equal_links_target_at_base():
    arm = TwoLinkArm(1, 1)
    t1, t2 = arm.ik(0, 0)
    _, tip = arm.fk(t1, t2)
    assert tip == pytest.approx([0, 0], abs=1e-9)


def test_nearest_reachable():
    assert ARM.nearest_reachable(7, 0) == pytest.approx((3.5, 0))
    assert ARM.nearest_reachable(0.1, 0) == pytest.approx((0.5, 0))
    assert ARM.nearest_reachable(1, 1) == pytest.approx((1, 1))


def test_random_property():
    rng = np.random.default_rng(1)
    for _ in range(200):
        r = math.sqrt(rng.uniform(ARM.min_reach**2, ARM.max_reach**2))
        a = rng.uniform(-math.pi, math.pi)
        x, y = r * math.cos(a), r * math.sin(a)
        t1, t2 = ARM.ik(x, y)
        _, tip = ARM.fk(t1, t2)
        assert tip == pytest.approx([x, y], abs=1e-9)
