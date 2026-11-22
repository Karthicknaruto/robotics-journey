"""ik.py: analytic inverse kinematics for a 2-link planar arm."""

import math

import numpy as np

from robotmath import rot2d

TOL = 1e-9  # numerical slack for targets that sit exactly on a reach limit


class UnreachableError(ValueError):
    """Raised when a target cannot be reached by the arm."""


def _check_lengths(L1, L2):
    if not (L1 > 0 and L2 > 0):
        raise ValueError(f"Link lengths must be positive, got L1={L1}, L2={L2}")


def reach_limits(L1, L2):
    """Return (min_reach, max_reach) for the arm."""
    _check_lengths(L1, L2)
    return abs(L1 - L2), L1 + L2


def is_reachable(x, y, L1, L2):
    """True if the target lies in the ring the tip can reach."""
    r_min, r_max = reach_limits(L1, L2)
    r = math.hypot(x, y)
    return (r_min - TOL) <= r <= (r_max + TOL)


def ik_2link(x, y, L1, L2, elbow_up=False):
    """Joint angles (theta1, theta2) in radians for a target (x, y).

    theta2 is measured relative to link 1 (counterclockwise positive).
    elbow_up=False gives theta2 >= 0 (elbow-down), elbow_up=True gives theta2 <= 0.
    Raises UnreachableError if the target is outside the reachable ring.
    """
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError(f"Target must be finite numbers, got ({x}, {y})")
    r_min, r_max = reach_limits(L1, L2)
    r_sq = x * x + y * y
    r = math.sqrt(r_sq)

    if r > r_max + TOL:
        raise UnreachableError(
            f"Target ({x}, {y}) is out of reach: distance {r:.4f} m "
            f"> maximum {r_max:.4f} m"
        )
    if r < r_min - TOL:
        raise UnreachableError(
            f"Target ({x}, {y}) is inside the dead zone: distance {r:.4f} m "
            f"< minimum {r_min:.4f} m"
        )

    # Law of cosines for the elbow angle. clip guards against rounding at the limits.
    cos_t2 = (r_sq - L1**2 - L2**2) / (2.0 * L1 * L2)
    cos_t2 = min(1.0, max(-1.0, cos_t2))
    theta2 = -math.acos(cos_t2) if elbow_up else math.acos(cos_t2)

    # Equal links and a target at the base: theta1 is arbitrary, so we choose 0.
    if r < TOL:
        return 0.0, theta2

    alpha = math.atan2(y, x)
    beta = math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
    return alpha - beta, theta2


def fk_2link(theta1, theta2, L1, L2):
    """Forward kinematics (radians): returns (elbow, tip). Used to check IK."""
    elbow = rot2d(theta1) @ np.array([L1, 0.0])
    tip = elbow + rot2d(theta1 + theta2) @ np.array([L2, 0.0])
    return elbow, tip


def nearest_reachable(x, y, L1, L2):
    """Closest point inside the reachable ring, along the line to the target."""
    r_min, r_max = reach_limits(L1, L2)
    r = math.hypot(x, y)
    ux, uy = (x / r, y / r) if r > 0 else (1.0, 0.0)
    r_clamped = min(max(r, r_min), r_max)
    return ux * r_clamped, uy * r_clamped
