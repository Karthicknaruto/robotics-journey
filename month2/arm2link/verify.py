"""verify.py: check IK by feeding its answers back through FK."""

import numpy as np

from .kinematics import UnreachableError


def random_reachable_targets(arm, n, rng):
    """n random targets spread evenly over the reachable ring (by AREA)."""
    r = np.sqrt(rng.uniform(arm.min_reach**2, arm.max_reach**2, n))  # sqrt: uniform density
    a = rng.uniform(-np.pi, np.pi, n)
    return np.column_stack([r * np.cos(a), r * np.sin(a)])


def ik_fk_error(arm, target, elbow_up):
    """Distance between the target and where FK says the IK answer lands."""
    t1, t2 = arm.ik(target[0], target[1], elbow_up=elbow_up)
    _, tip = arm.fk(t1, t2)
    return float(np.linalg.norm(tip - np.asarray(target)))


def verify_ik(arm, n=100, seed=42, n_wide=1000):
    """Run the full verification and return the results as a dictionary."""
    rng = np.random.default_rng(seed)
    targets = random_reachable_targets(arm, n, rng)
    err_down = np.array([ik_fk_error(arm, t, False) for t in targets])
    err_up = np.array([ik_fk_error(arm, t, True) for t in targets])

    # Random points in a square much bigger than the reach: count wrong decisions.
    side = 1.4 * arm.max_reach
    wide = rng.uniform(-side, side, size=(n_wide, 2))
    solved = wrongly_rejected = wrongly_accepted = 0
    for x, y in wide:
        reachable = arm.is_reachable(x, y)
        try:
            arm.ik(x, y)
            solved += 1
            if not reachable:
                wrongly_accepted += 1
        except UnreachableError:
            if reachable:
                wrongly_rejected += 1

    err_all = np.concatenate([err_down, err_up])
    return {
        "targets": targets,
        "err_down": err_down,
        "err_up": err_up,
        "err_all": err_all,
        "n": n,
        "max_error": float(err_all.max()),
        "mean_error": float(err_all.mean()),
        "n_wide": n_wide,
        "wide_solved": solved,
        "wrongly_accepted": wrongly_accepted,
        "wrongly_rejected": wrongly_rejected,
    }
