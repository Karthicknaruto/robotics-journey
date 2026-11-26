"""paths.py: build paths and turn them into joint angles with IK."""

import numpy as np

from .kinematics import UnreachableError


def circle_path(center, radius, n):
    """n+1 points around a circle; the last point equals the first (closed loop)."""
    t = np.linspace(0, 2 * np.pi, n + 1)
    return np.column_stack([center[0] + radius * np.cos(t),
                            center[1] + radius * np.sin(t)])


def line_path(start, end, n):
    """n points evenly spaced on a straight line."""
    return np.linspace(start, end, n)


def solve_path(arm, path, elbow_up=False):
    """Joint angles (radians) for every waypoint: an (N, 2) array.

    Every waypoint is checked first, so a bad path fails before any work is done.
    theta1 is unwrapped to remove artificial 360-degree jumps.
    """
    for i, (x, y) in enumerate(path):
        if not arm.is_reachable(x, y):
            raise UnreachableError(f"Waypoint {i} at ({x:.3f}, {y:.3f}) cannot be reached")
    angles = np.array([arm.ik(x, y, elbow_up=elbow_up) for x, y in path])
    angles[:, 0] = np.unwrap(angles[:, 0])
    return angles


def tracking_error(arm, path, angles):
    """Distance between each waypoint and where FK says the tip ends up (metres)."""
    tips = arm.tip_positions(angles[:, 0], angles[:, 1])
    return np.linalg.norm(tips - path, axis=1)


def joint_steps_deg(angles):
    """Absolute change of each joint between consecutive waypoints, in degrees."""
    return np.abs(np.diff(np.degrees(angles), axis=0))
