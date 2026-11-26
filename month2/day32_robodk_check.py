"""Day 32: check that your SE(3) code matches RoboDK's pose maths, then talk to RoboDK.

Setup:  pip install robodk numpy
Part A works anywhere (no RoboDK needed). Part B needs RoboDK open with a robot loaded.
Needs day31_se3.py in the same folder.
"""
import sys

import numpy as np
from robodk import robomath as rm

from day31_se3 import make_T, rot_x, rot_y, rot_z

ANGLE = np.radians(35)


def to_numpy(mat):
    """Convert a RoboDK Mat (4x4 pose) to a numpy array."""
    return np.array([[mat[i, j] for j in range(4)] for i in range(4)], dtype=float)


def part_a_offline():
    """Compare my rotations and transforms with RoboDK's robomath."""
    print("Part A: my SE(3) code versus RoboDK's robomath")
    for name, mine, theirs in [("rot_x", rot_x, rm.rotx), ("rot_y", rot_y, rm.roty), ("rot_z", rot_z, rm.rotz)]:
        a = make_T(mine(ANGLE), [0, 0, 0])
        b = to_numpy(theirs(ANGLE))
        print(f"  {name}(35 deg) matches RoboDK: {np.allclose(a, b)}")

    # A pose built as: move to (100, 200, 300) mm, then turn 35 deg about z
    mine = make_T(np.eye(3), [100, 200, 300]) @ make_T(rot_z(ANGLE), [0, 0, 0])
    theirs = to_numpy(rm.transl(100, 200, 300) * rm.rotz(ANGLE))
    print(f"  chained pose matches RoboDK   : {np.allclose(mine, theirs)}")

    # RoboDK's inverse versus mine
    from day31_se3 import inv_T
    print(f"  inverse matches RoboDK        : {np.allclose(inv_T(mine), to_numpy(rm.invH(rm.transl(100, 200, 300) * rm.rotz(ANGLE))))}")

    # RoboDK shows poses as X, Y, Z (mm) and rx, ry, rz (degrees)
    pose = rm.transl(100, 200, 300) * rm.rotz(ANGLE)
    print("  RoboDK display of that pose   :", str(pose).splitlines()[0])


def part_b_online():
    """Connect to RoboDK and print the first robot's joints and flange pose."""
    print("\nPart B: talk to RoboDK")
    try:
        from robodk.robolink import ITEM_TYPE_ROBOT, Robolink
        rdk = Robolink()
        robots = rdk.ItemList(ITEM_TYPE_ROBOT)
        if not robots:
            print("  RoboDK is open but no robot is loaded. Add one from File > Open online library.")
            return
        robot = robots[0]
        print("  Robot name :", robot.Name())
        print("  Joints (deg):", np.round(np.array(robot.Joints().list()), 3))
        print("  Flange pose in the robot base frame:")
        print(np.round(to_numpy(robot.Pose()), 3))
    except Exception as err:  # RoboDK not installed or not running
        print("  Could not reach RoboDK:", type(err).__name__, "-", err)
        print("  Open RoboDK, load a robot, then run this file again.")


if __name__ == "__main__":
    part_a_offline()
    if "--offline" not in sys.argv:
        part_b_online()