"""Day 30: DOF of a four-bar, a serial arm and a delta robot, plus a numeric four-bar check.

Needs day29_dof.py in the same folder.
"""
import math

import numpy as np

from day29_dof import grubler


# ---------- Part 1: DOF by Grubler ----------

def four_bar_dof():
    """Ground + crank + coupler + rocker = 4 links, 4 revolute joints (planar)."""
    return grubler(4, ["revolute"] * 4, planar=True)


def serial_arm_dof(n_joints=6):
    """Ground + one link per joint, all revolute (3D)."""
    return grubler(n_joints + 1, ["revolute"] * n_joints, planar=False)


def delta_robot_dof():
    """Delta robot modelled with spherical joints at the rod ends.

    Links : base + platform + 3 upper arms + 6 rods (2 per leg) = 11
    Joints: 3 revolute (motors) + 12 spherical (rod ends) = 15
    Grubler counts 6 extra 'idle' freedoms: each rod can spin about its own axis.
    Those spins do not move the platform, so they are subtracted.

    Returns (grubler_value, idle_spins, real_dof).
    """
    n_links = 1 + 1 + 3 + 6
    joints = ["revolute"] * 3 + ["spherical"] * 12
    raw = grubler(n_links, joints, planar=False)
    idle = 6
    return raw, idle, raw - idle


def braced_triangle_dof():
    """Triangle (ground is one bar) plus one extra bar pinned across it: N=4, J=5."""
    return grubler(4, ["revolute"] * 5, planar=True)


# ---------- Part 2: numeric check of the four-bar ----------

def four_bar_rocker(theta, ground=4.0, crank=1.5, coupler=4.0, rocker=3.5):
    """For a crank angle, return (B, C, rocker_angle) or None if the linkage cannot close.

    A = (0, 0) and D = (ground, 0) are the fixed pivots.
    """
    A = np.array([0.0, 0.0])
    D = np.array([ground, 0.0])
    B = A + crank * np.array([math.cos(theta), math.sin(theta)])
    bd = D - B
    d = np.linalg.norm(bd)
    a1 = (coupler**2 - rocker**2 + d**2) / (2 * d)
    h_sq = coupler**2 - a1**2
    if h_sq < 0:
        return None
    h = math.sqrt(h_sq)
    u = bd / d
    perp = np.array([-u[1], u[0]])
    C = B + a1 * u - h * perp  # one fixed branch of the two possible closings
    rocker_angle = math.atan2(C[1] - D[1], C[0] - D[0])
    return B, C, rocker_angle


def main():
    print("DOF by Grubler's formula:")
    print(f"  Four-bar linkage            : {four_bar_dof()}")
    print(f"  Serial arm (6 revolute)     : {serial_arm_dof(6)}")
    print(f"  Serial arm (3 revolute)     : {serial_arm_dof(3)}")
    raw, idle, real = delta_robot_dof()
    print(f"  Delta robot (Grubler value) : {raw}")
    print(f"    minus idle rod spins      : {idle}")
    print(f"    real DOF of the platform  : {real}")
    print(f"  Braced triangle (N=4, J=5)  : {braced_triangle_dof()}  (negative = over-constrained)")

    print("\nNumeric check of the four-bar (ground 4.0, crank 1.5, coupler 4.0, rocker 3.5):")
    lengths_ok = True
    angles = []
    for deg in range(0, 360, 5):
        res = four_bar_rocker(math.radians(deg))
        if res is None:
            lengths_ok = False
            print(f"  crank {deg} deg: linkage cannot close")
            continue
        B, C, ang = res
        # The coupler and rocker lengths must hold at every crank angle
        assert abs(np.linalg.norm(C - B) - 4.0) < 1e-9
        assert abs(np.linalg.norm(C - np.array([4.0, 0.0])) - 3.5) < 1e-9
        angles.append(math.degrees(ang))
    print(f"  crank turns fully (72 angles tested): {lengths_ok}")
    print(f"  rocker swings from {min(angles):.1f} to {max(angles):.1f} deg "
          f"(range {max(angles) - min(angles):.1f} deg)")
    print("  One crank angle fixes the whole linkage -> 1 DOF, as Grubler predicted.")


if __name__ == "__main__":
    main()