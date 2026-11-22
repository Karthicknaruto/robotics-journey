import pytest

from arm2link import TwoLinkArm, sample_workspace, verify_ik, workspace_summary
import numpy as np

ARM = TwoLinkArm(2.0, 1.5)


def test_samples_stay_inside_ring():
    _, tips = sample_workspace(ARM, n=5000, seed=3)
    r = np.hypot(tips[:, 0], tips[:, 1])
    assert r.min() >= ARM.min_reach - 1e-9
    assert r.max() <= ARM.max_reach + 1e-9


def test_same_seed_same_samples():
    _, a = sample_workspace(ARM, n=100, seed=5)
    _, b = sample_workspace(ARM, n=100, seed=5)
    assert np.array_equal(a, b)


def test_summary_area():
    _, tips = sample_workspace(ARM, n=1000)
    s = workspace_summary(ARM, tips)
    assert s["area_m2"] == pytest.approx(np.pi * (3.5**2 - 0.5**2))


def test_verify_ik_small():
    res = verify_ik(ARM, n=30, seed=1, n_wide=300)
    assert res["max_error"] < 1e-9
    assert res["wrongly_accepted"] == 0
    assert res["wrongly_rejected"] == 0
