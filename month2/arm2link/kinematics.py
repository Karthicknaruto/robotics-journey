"""kinematics.py: forward and inverse kinematics of a 2-link planar arm."""

import math
from dataclasses import dataclass

import numpy as np

from .transforms import link_transform

TOL = 1e-9  # numerical slack for targets that sit exactly on a reach limit


class UnreachableError(ValueError):
    """Raised when a target cannot be reached by the arm."""


@dataclass(frozen=True)
class TwoLinkArm:
    """A planar arm with two revolute joints, base at the origin.

    theta1 is measured from the x-axis. theta2 is measured relative to link 1,
    counterclockwise positive.
    """

    L1: float
    L2: float

    def __post_init__(self):
        if not (self.L1 > 0 and self.L2 > 0):
            raise ValueError(
                f"Link lengths must be positive, got L1={self.L1}, L2={self.L2}"
            )

    # ----- reach -----
    @property
    def max_reach(self):
        """Farthest distance the tip can be from the base (arm stretched)."""
        return self.L1 + self.L2

    @property
    def min_reach(self):
        """Closest distance the tip can be to the base (arm folded)."""
        return abs(self.L1 - self.L2)

    def is_reachable(self, x, y):
        """True if the target lies in the ring the tip can reach."""
        r = math.hypot(x, y)
        return (self.min_reach - TOL) <= r <= (self.max_reach + TOL)

    def nearest_reachable(self, x, y):
        """Closest reachable point, along the line from the base to the target."""
        r = math.hypot(x, y)
        ux, uy = (x / r, y / r) if r > 0 else (1.0, 0.0)
        r_clamped = min(max(r, self.min_reach), self.max_reach)
        return ux * r_clamped, uy * r_clamped

    # ----- forward kinematics -----
    def fk(self, theta1, theta2):
        """Return (elbow, tip) positions by chaining link transforms (radians)."""
        T1 = link_transform(theta1, self.L1)  # frame at the elbow
        T2 = link_transform(theta2, self.L2)  # frame at the tip, seen from the elbow
        elbow = T1[:2, 3]
        tip = (T1 @ T2)[:2, 3]
        return elbow, tip

    def tip_positions(self, theta1, theta2):
        """Vectorised closed-form FK: arrays of angles in, (N, 2) tip positions out."""
        t1 = np.asarray(theta1, dtype=float)
        t2 = np.asarray(theta2, dtype=float)
        x = self.L1 * np.cos(t1) + self.L2 * np.cos(t1 + t2)
        y = self.L1 * np.sin(t1) + self.L2 * np.sin(t1 + t2)
        return np.column_stack([x, y])

    # ----- inverse kinematics -----
    def ik(self, x, y, elbow_up=False):
        """Joint angles (theta1, theta2) in radians that put the tip at (x, y).

        elbow_up=False gives theta2 >= 0 (elbow-down), elbow_up=True gives theta2 <= 0.
        Raises UnreachableError if the target is outside the reachable ring.
        """
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f"Target must be finite numbers, got ({x}, {y})")
        r_sq = x * x + y * y
        r = math.sqrt(r_sq)

        if r > self.max_reach + TOL:
            raise UnreachableError(
                f"Target ({x}, {y}) is out of reach: distance {r:.4f} m "
                f"> maximum {self.max_reach:.4f} m"
            )
        if r < self.min_reach - TOL:
            raise UnreachableError(
                f"Target ({x}, {y}) is inside the dead zone: distance {r:.4f} m "
                f"< minimum {self.min_reach:.4f} m"
            )

        # Law of cosines for the elbow angle; the clamp guards against rounding.
        cos_t2 = (r_sq - self.L1**2 - self.L2**2) / (2.0 * self.L1 * self.L2)
        cos_t2 = min(1.0, max(-1.0, cos_t2))
        theta2 = -math.acos(cos_t2) if elbow_up else math.acos(cos_t2)

        # Equal links and a target at the base: theta1 is arbitrary, so choose 0.
        if r < TOL:
            return 0.0, theta2

        alpha = math.atan2(y, x)  # direction of the target
        beta = math.atan2(self.L2 * math.sin(theta2), self.L1 + self.L2 * math.cos(theta2))
        return alpha - beta, theta2
