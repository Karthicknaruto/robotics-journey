"""transforms.py: small rotation and transform helpers."""

import numpy as np


def deg2rad(deg):
    """Convert degrees to radians."""
    return np.deg2rad(deg)


def rot2d(theta_rad):
    """2x2 rotation matrix (counterclockwise)."""
    c, s = np.cos(theta_rad), np.sin(theta_rad)
    return np.array([[c, -s],
                     [s, c]])


def rot_z(theta_rad):
    """3x3 rotation about the z-axis."""
    c, s = np.cos(theta_rad), np.sin(theta_rad)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]])


def make_transform(R, t):
    """4x4 homogeneous transform from a 3x3 rotation and a translation 3-vector."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def link_transform(theta_rad, length):
    """Frame at the END of a link, seen from the frame at its START.

    Step 1: rotate by theta at the joint.
    Step 2: move `length` along the new x-axis to the next joint.
    """
    rotate = make_transform(rot_z(theta_rad), [0, 0, 0])
    move = make_transform(np.eye(3), [length, 0, 0])
    return rotate @ move
