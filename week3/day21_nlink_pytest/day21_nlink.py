"""day21_nlink.py: draw N-link arms (2, 3 and 5 links) with the loop-based FK."""

import os

import matplotlib.pyplot as plt
import numpy as np

from nlink import fk_nlink, max_reach, tip_closed_form

HERE = os.path.dirname(os.path.abspath(__file__))


def draw_arm(ax, angles_deg, lengths, title):
    pts = fk_nlink(np.radians(angles_deg), lengths)
    reach = max_reach(lengths)
    ring = np.linspace(0, 2 * np.pi, 200)
    ax.plot(reach * np.cos(ring), reach * np.sin(ring), "k--", alpha=0.25)
    ax.plot(pts[:, 0], pts[:, 1], "-o", color="tab:blue", lw=3)
    ax.plot(*pts[-1], "r*", markersize=15)
    ax.set_xlim(-reach - 0.5, reach + 0.5)
    ax.set_ylim(-reach - 0.5, reach + 0.5)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="k", lw=0.5)
    ax.axvline(0, color="k", lw=0.5)
    ax.set_title(f"{title}\ntip = ({pts[-1][0]:.2f}, {pts[-1][1]:.2f})", fontsize=10)


def main():
    np.set_printoptions(suppress=True)

    # Worked example: 3 links, abs angles 30, 90, 45
    angles = np.radians([30, 60, -45])
    lengths = [2.0, 1.5, 1.0]
    pts = fk_nlink(angles, lengths)
    print("3-link joint positions:\n", np.round(pts, 4))
    print("Loop tip       :", np.round(pts[-1], 4))
    print("Closed-form tip:", np.round(tip_closed_form(angles, lengths), 4))
    print("Max reach      :", max_reach(lengths), "m")

    fig, axs = plt.subplots(1, 3, figsize=(15, 5.5))
    draw_arm(axs[0], [30, 60], [2.0, 1.5], "2 links: 30, 60 deg")
    draw_arm(axs[1], [30, 60, -45], [2.0, 1.5, 1.0], "3 links: 30, 60, -45 deg")
    draw_arm(axs[2], [20, 25, -30, 40, -50], [1.5, 1.2, 1.0, 0.8, 0.5],
             "5 links: 20, 25, -30, 40, -50 deg")
    fig.suptitle("Day 21: N-link planar arms")
    fig.tight_layout()
    path = os.path.join(HERE, "day21_nlink_arms.png")
    fig.savefig(path, dpi=200, bbox_inches="tight")
    print("Saved", path)
    plt.show()


if __name__ == "__main__":
    main()
