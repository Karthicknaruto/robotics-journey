"""day22_ik_check.py: check the paper derivation of 2-link inverse kinematics."""

import os

import matplotlib.pyplot as plt
import numpy as np

from robotmath import rot2d

HERE = os.path.dirname(os.path.abspath(__file__))
L1, L2 = 2.0, 1.5


def fk(t1, t2):
    """Forward kinematics (radians): returns elbow and tip."""
    elbow = rot2d(t1) @ np.array([L1, 0.0])
    tip = elbow + rot2d(t1 + t2) @ np.array([L2, 0.0])
    return elbow, tip


def ik_both(x, y):
    """Return {'elbow-down': (t1, t2), 'elbow-up': (t1, t2)} in radians, or None."""
    r_sq = x**2 + y**2
    r = np.sqrt(r_sq)
    if r > L1 + L2 or r < abs(L1 - L2):
        return None  # Day 23 turns this into proper error handling

    cos_t2 = (r_sq - L1**2 - L2**2) / (2 * L1 * L2)   # law of cosines
    cos_t2 = np.clip(cos_t2, -1.0, 1.0)
    alpha = np.arctan2(y, x)                           # direction to the target

    solutions = {}
    for name, t2 in (("elbow-down", np.arccos(cos_t2)),
                     ("elbow-up", -np.arccos(cos_t2))):
        beta = np.arctan2(L2 * np.sin(t2), L1 + L2 * np.cos(t2))
        solutions[name] = (alpha - beta, t2)
    return solutions


def main():
    np.set_printoptions(suppress=True)
    targets = [(1.8, 2.2), (-1.0, 1.5), (3.5, 0.0), (0.5, 0.0), (4.0, 0.0)]

    for x, y in targets:
        print(f"\nTarget ({x}, {y}),  distance = {np.hypot(x, y):.4f} m")
        sols = ik_both(x, y)
        if sols is None:
            print("  Out of reach (needs 0.5 <= distance <= 3.5)")
            continue
        for name, (t1, t2) in sols.items():
            _, tip = fk(t1, t2)
            err = np.linalg.norm(tip - np.array([x, y]))
            print(f"  {name:10s}: t1 = {np.degrees(t1):8.2f} deg, "
                  f"t2 = {np.degrees(t2):8.2f} deg, FK check error = {err:.1e}")

    # Plot both solutions for the first target
    x, y = targets[0]
    sols = ik_both(x, y)
    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    ring = np.linspace(0, 2 * np.pi, 200)
    ax.plot((L1 + L2) * np.cos(ring), (L1 + L2) * np.sin(ring), "k--", alpha=0.25)
    ax.plot(abs(L1 - L2) * np.cos(ring), abs(L1 - L2) * np.sin(ring), "k:", alpha=0.25)
    styles = {"elbow-down": ("-o", "tab:blue"), "elbow-up": ("--o", "tab:red")}
    for name, (t1, t2) in sols.items():
        elbow, tip = fk(t1, t2)
        fmt, color = styles[name]
        ax.plot([0, elbow[0], tip[0]], [0, elbow[1], tip[1]], fmt,
                color=color, lw=3, label=name)
    ax.plot([0, x], [0, y], ":", color="gray")
    ax.plot(x, y, "g*", markersize=18, label="target")
    ax.set_xlim(-4, 4)
    ax.set_ylim(-4, 4)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left")
    ax.set_title("Day 22: two IK solutions for one target")
    path = os.path.join(HERE, "day22_two_solutions.png")
    fig.savefig(path, dpi=200, bbox_inches="tight")
    print("\nSaved", path)
    plt.show()


if __name__ == "__main__":
    main()
