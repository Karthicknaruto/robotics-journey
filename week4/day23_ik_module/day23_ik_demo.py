"""day23_ik_demo.py: use ik.py on good and bad targets, and plot what happens."""

import os

import matplotlib.pyplot as plt
import numpy as np

from ik import (UnreachableError, fk_2link, ik_2link, nearest_reachable,
                reach_limits)

HERE = os.path.dirname(os.path.abspath(__file__))
L1, L2 = 2.0, 1.5


def draw(ax, t1, t2, color, label, ls="-"):
    elbow, tip = fk_2link(t1, t2, L1, L2)
    ax.plot([0, elbow[0], tip[0]], [0, elbow[1], tip[1]], ls + "o",
            color=color, lw=3, label=label)


def main():
    r_min, r_max = reach_limits(L1, L2)
    targets = {
        "reachable": (1.8, 2.2),
        "too far": (4.0, 2.0),
        "dead zone": (-0.2, 0.15),
    }

    for name, (x, y) in targets.items():
        print(f"\n{name}: target ({x}, {y})")
        try:
            t1, t2 = ik_2link(x, y, L1, L2)
            print(f"  OK: theta1 = {np.degrees(t1):.2f} deg, theta2 = {np.degrees(t2):.2f} deg")
        except UnreachableError as err:
            print("  UnreachableError:", err)
            cx, cy = nearest_reachable(x, y, L1, L2)
            print(f"  Nearest reachable point: ({cx:.4f}, {cy:.4f})")

    fig, ax = plt.subplots(figsize=(7.5, 7.5))
    ring = np.linspace(0, 2 * np.pi, 300)
    ax.plot(r_max * np.cos(ring), r_max * np.sin(ring), "k--", alpha=0.35,
            label=f"max reach {r_max} m")
    ax.plot(r_min * np.cos(ring), r_min * np.sin(ring), "k:", alpha=0.5,
            label=f"min reach {r_min} m")

    colors = {"reachable": "tab:green", "too far": "tab:red", "dead zone": "tab:orange"}
    for name, (x, y) in targets.items():
        ax.plot(x, y, "*", color=colors[name], markersize=16)
        ax.annotate(name, (x, y), textcoords="offset points", xytext=(10, 10) if x > 0 else (-70, 14),
                    color=colors[name])
        try:
            t1, t2 = ik_2link(x, y, L1, L2)
            draw(ax, t1, t2, colors[name], "arm at reachable target")
        except UnreachableError:
            cx, cy = nearest_reachable(x, y, L1, L2)
            t1, t2 = ik_2link(cx, cy, L1, L2)
            draw(ax, t1, t2, colors[name], f"arm at nearest point ({name})", ls="--")

    ax.set_xlim(-4, 5)
    ax.set_ylim(-4, 4)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower left", fontsize=8)
    ax.set_title("Day 23: reachable, too far and dead-zone targets")
    path = os.path.join(HERE, "day23_reach_handling.png")
    fig.savefig(path, dpi=200, bbox_inches="tight")
    print("\nSaved", path)
    plt.show()


if __name__ == "__main__":
    main()
