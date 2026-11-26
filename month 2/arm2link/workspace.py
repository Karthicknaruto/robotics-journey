"""workspace.py: sample joint angles to find where the tip can go."""

import numpy as np


def sample_workspace(arm, n=20000, seed=0):
    """Pick n random joint-angle pairs and return (angles, tips).

    Both joints are sampled uniformly from -180 to 180 degrees.
    """
    rng = np.random.default_rng(seed)
    angles = rng.uniform(-np.pi, np.pi, size=(n, 2))
    tips = arm.tip_positions(angles[:, 0], angles[:, 1])
    return angles, tips


def workspace_summary(arm, tips):
    """Compare the reach seen in the samples with the theoretical limits."""
    r = np.hypot(tips[:, 0], tips[:, 1])
    area = np.pi * (arm.max_reach**2 - arm.min_reach**2)
    return {
        "samples": len(tips),
        "observed_min_reach": float(r.min()),
        "observed_max_reach": float(r.max()),
        "theory_min_reach": arm.min_reach,
        "theory_max_reach": arm.max_reach,
        "area_m2": float(area),
    }
