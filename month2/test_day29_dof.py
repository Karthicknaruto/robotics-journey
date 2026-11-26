import pytest

from day29_dof import cspace_shape, grubler


@pytest.mark.parametrize("n,joints,planar,dof", [
    (3, ["revolute"] * 2, True, 2),
    (4, ["revolute"] * 4, True, 1),
    (5, ["revolute"] * 5, True, 2),
    (3, ["revolute"] * 3, True, 0),
    (7, ["revolute"] * 6, False, 6),
    (14, ["universal"] * 6 + ["prismatic"] * 6 + ["spherical"] * 6, False, 6),
])
def test_known_mechanisms(n, joints, planar, dof):
    assert grubler(n, joints, planar) == dof


def test_unknown_joint():
    with pytest.raises(ValueError):
        grubler(3, ["spherical"], planar=True)


def test_shapes():
    assert cspace_shape(["revolute"] * 2) == "S1^2"
    assert cspace_shape(["prismatic"] * 3) == "R^3"