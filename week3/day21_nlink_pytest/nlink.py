"""nlink.py: forward kinematics for a planar arm with any number of links."""

import numpy as np

from robotmath import link_transform


def _check_inputs(angles_rad, lengths):
    if len(angles_rad) != len(lengths):
        raise ValueError(
            f"Need one angle per link: got {len(angles_rad)} angles "
            f"and {len(lengths)} lengths"
        )
    if len(lengths) == 0:
        raise ValueError("The arm needs at least one link")
    if any(L <= 0 for L in lengths):
        raise ValueError("Link lengths must be positive")


def fk_nlink(angles_rad, lengths):
    """Return joint positions as an (N+1, 2) array: base, joint 1, ..., tip.

    Loop idea: keep one running transform T (the frame at the end of the chain
    so far). For every link, multiply by that link's transform.
    """
    _check_inputs(angles_rad, lengths)
    T = np.eye(4)
    points = [T[:2, 3].copy()]  # the base
    for theta, length in zip(angles_rad, lengths):
        T = T @ link_transform(theta, length)
        points.append(T[:2, 3].copy())
    return np.array(points)


def tip_closed_form(angles_rad, lengths):
    """Same answer from the paper formula: add up L_i at the running angle."""
    _check_inputs(angles_rad, lengths)
    absolute = np.cumsum(angles_rad)  # theta1, theta1+theta2, theta1+theta2+theta3, ...
    lengths = np.asarray(lengths, dtype=float)
    return np.array([np.sum(lengths * np.cos(absolute)),
                     np.sum(lengths * np.sin(absolute))])


def max_reach(lengths):
    """Farthest the tip can be from the base: all links stretched in a line."""
    return float(np.sum(lengths))
