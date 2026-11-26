import numpy as np
import pytest

from day31_se3 import (inv_T, is_se3, is_so3, make_T, random_so3, rot_x, rot_y,
                       rot_z, transform_point)


@pytest.mark.parametrize("f", [rot_x, rot_y, rot_z])
@pytest.mark.parametrize("deg", [0, 30, 90, 179, -45])
def test_basic_rotations_valid(f, deg):
    assert is_so3(f(np.radians(deg)))


def test_reflection_rejected():
    assert not is_so3(np.diag([-1, 1, 1]))


def test_scale_rejected():
    assert not is_so3(2 * np.eye(3))


def test_wrong_shape_rejected():
    assert not is_so3(np.eye(4))


def test_random_rotations_valid_and_closed():
    rng = np.random.default_rng(3)
    for _ in range(200):
        A, B = random_so3(rng), random_so3(rng)
        assert is_so3(A) and is_so3(A @ B)


def test_order_matters():
    A, B = rot_x(np.pi / 2), rot_z(np.pi / 2)
    assert not np.allclose(A @ B, B @ A)


def test_se3_validity():
    T = make_T(rot_z(0.4), [1, 2, 3])
    assert is_se3(T)
    bad = T.copy()
    bad[3, 3] = 2
    assert not is_se3(bad)


def test_inverse():
    rng = np.random.default_rng(5)
    for _ in range(100):
        T = make_T(random_so3(rng), rng.normal(size=3))
        assert np.allclose(T @ inv_T(T), np.eye(4))
        assert np.allclose(inv_T(T) @ T, np.eye(4))


def test_distance_preserved():
    rng = np.random.default_rng(7)
    T = make_T(random_so3(rng), rng.normal(size=3))
    p, q = rng.normal(size=3), rng.normal(size=3)
    d0 = np.linalg.norm(p - q)
    d1 = np.linalg.norm(transform_point(T, p) - transform_point(T, q))
    assert d1 == pytest.approx(d0)


def test_point_example():
    T = make_T(rot_z(np.pi / 2), [1, 0, 0])
    assert transform_point(T, [1, 0, 0]) == pytest.approx([1, 1, 0])