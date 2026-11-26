"""Day 29: degrees of freedom (Grubler) and the shape of configuration space.

Standalone: needs only the Python standard library.
"""

# Freedom of each joint type. Planar joints live in a plane, spatial joints in 3D.
PLANAR_JOINTS = {"revolute": 1, "prismatic": 1}
SPATIAL_JOINTS = {
    "revolute": 1,
    "prismatic": 1,
    "helical": 1,
    "cylindrical": 2,
    "universal": 2,
    "spherical": 3,
}

# Shape of the space of one joint: a circle, a straight line, or a sphere.
JOINT_SHAPE = {
    "revolute": "S1",     # angle wraps around: circle
    "prismatic": "R",     # slides along a line
    "helical": "R",
    "cylindrical": "S1xR",
    "universal": "S1xS1",
    "spherical": "S2xS1",  # 3 DOF: a direction (sphere) plus a spin
}


def grubler(n_links, joints, planar=True):
    """Degrees of freedom of a mechanism.

    n_links : number of links INCLUDING the ground (fixed base)
    joints  : list of joint type names, one entry per joint
    planar  : True for a mechanism moving in a plane, False for 3D
    """
    table = PLANAR_JOINTS if planar else SPATIAL_JOINTS
    for j in joints:
        if j not in table:
            raise ValueError(f"Unknown {'planar' if planar else 'spatial'} joint: {j!r}")
    m = 3 if planar else 6
    return m * (n_links - 1 - len(joints)) + sum(table[j] for j in joints)


def cspace_shape(joints):
    """Describe the shape of the configuration space of an open serial chain."""
    from collections import Counter

    counts = Counter(JOINT_SHAPE[j] for j in joints)
    return " x ".join(f"{s}^{c}" if c > 1 else s for s, c in counts.items())


def report(name, dof, expected=None):
    flag = ""
    if expected is not None:
        flag = "  OK" if dof == expected else f"  MISMATCH (expected {expected})"
    print(f"  {name:<34}{dof:>3}{flag}")


def main():
    print("Planar mechanisms (3 x (N - 1 - J) + sum of joint freedoms):")
    report("2R arm (N=3, J=2)", grubler(3, ["revolute"] * 2), 2)
    report("3R arm (N=4, J=3)", grubler(4, ["revolute"] * 3), 3)
    report("Four-bar linkage (N=4, J=4)", grubler(4, ["revolute"] * 4), 1)
    report("Slider-crank (N=4, 3R + 1P)", grubler(4, ["revolute"] * 3 + ["prismatic"]), 1)
    report("Five-bar linkage (N=5, J=5)", grubler(5, ["revolute"] * 5), 2)
    report("Rigid triangle (N=3, J=3)", grubler(3, ["revolute"] * 3), 0)

    print("\nSpatial mechanisms (6 x (N - 1 - J) + sum of joint freedoms):")
    report("6R industrial arm (N=7, J=6)", grubler(7, ["revolute"] * 6, planar=False), 6)
    report("Arm + weld gun axis (N=8, 6R+1P)", grubler(8, ["revolute"] * 6 + ["prismatic"], planar=False), 7)
    stewart = ["universal"] * 6 + ["prismatic"] * 6 + ["spherical"] * 6
    report("Stewart platform (N=14, J=18)", grubler(14, stewart, planar=False), 6)

    print("\nShape of configuration space (open serial chains):")
    print("  2R planar arm     :", cspace_shape(["revolute"] * 2), "(a torus)")
    print("  6R industrial arm :", cspace_shape(["revolute"] * 6))
    print("  Gantry (3P)       :", cspace_shape(["prismatic"] * 3), "(a box of travel, no wrap)")
    print("  Slider + rotary   :", cspace_shape(["prismatic", "revolute"]), "(a cylinder surface)")

    # Wrap-around: two angles are the same pose if they differ by 360 degrees
    print("\nWrap-around check (angles in degrees):")
    for a, b in [(-179, 179), (10, 350), (0, 180)]:
        gap = abs((a - b + 180) % 360 - 180)
        print(f"  {a:>5} and {b:>4}: naive gap {abs(a - b):>3}, true gap {gap:>3}")


if __name__ == "__main__":
    main()