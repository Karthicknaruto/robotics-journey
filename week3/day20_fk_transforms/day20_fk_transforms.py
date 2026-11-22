"""day20_fk_transforms.py: 2-link forward kinematics using transforms, with plots."""

import os

import matplotlib.pyplot as plt
import numpy as np

from robotmath import deg2rad, link_transform

HERE = os.path.dirname(os.path.abspath(__file__))
L1, L2 = 2.0, 1.5  # link lengths in metres


def fk_2link(t1_rad, t2_rad, l1=L1, l2=L2):
    """Return (base, elbow, tip) positions by chaining link transforms."""
    T1 = link_transform(t1_rad, l1)     # frame at the elbow, seen from the base
    T2 = link_transform(t2_rad, l2)     # frame at the tip, seen from the elbow
    T_tip = T1 @ T2                     # frame at the tip, seen from the base
    base = np.array([0.0, 0.0])
    elbow = T1[:2, 3]
    tip = T_tip[:2, 3]
    return base, elbow, tip


def fk_paper(t1_rad, t2_rad, l1=L1, l2=L2):
    """Closed-form formula from the Day 19 paper derivation."""
    x = l1 * np.cos(t1_rad) + l2 * np.cos(t1_rad + t2_rad)
    y = l1 * np.sin(t1_rad) + l2 * np.sin(t1_rad + t2_rad)
    return np.array([x, y])


def draw_reach(ax):
    ring = np.linspace(0, 2 * np.pi, 200)
    ax.plot((L1 + L2) * np.cos(ring), (L1 + L2) * np.sin(ring), "k--", alpha=0.25)
    if abs(L1 - L2) > 0.05:
        ax.plot(abs(L1 - L2) * np.cos(ring), abs(L1 - L2) * np.sin(ring),
                "k:", alpha=0.25)


def style(ax, title):
    lim = L1 + L2 + 0.5
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="k", lw=0.5)
    ax.axvline(0, color="k", lw=0.5)
    ax.set_title(title, fontsize=10)


def block1_poses():
    """Block 1: draw the arm for several angle pairs."""
    pairs = [(0, 0), (30, 60), (90, -90), (45, 45), (120, -60), (200, -45)]
    fig, axs = plt.subplots(2, 3, figsize=(12, 8))
    for ax, (a, b) in zip(axs.flat, pairs):
        base, elbow, tip = fk_2link(deg2rad(a), deg2rad(b))
        draw_reach(ax)
        ax.plot([base[0], elbow[0], tip[0]], [base[1], elbow[1], tip[1]],
                "-o", color="tab:blue", lw=3)
        ax.plot(*tip, "r*", markersize=14)
        style(ax, f"t1 = {a} deg, t2 = {b} deg\ntip = ({tip[0]:.2f}, {tip[1]:.2f})")
    fig.suptitle("Day 20: 2-link arm poses (forward kinematics with transforms)")
    fig.tight_layout()
    path = os.path.join(HERE, "day20_arm_poses.png")
    fig.savefig(path, dpi=200, bbox_inches="tight")
    return path


def block2_paths():
    """Block 2: tip path as the joints sweep, checked against the paper formula."""
    sweeps = {
        "Sweep t1: 0 to 180 deg (t2 = 60)": (np.linspace(0, np.pi, 100),
                                              np.full(100, deg2rad(60))),
        "Sweep t2: -150 to 150 deg (t1 = 30)": (np.full(100, deg2rad(30)),
                                                 np.linspace(deg2rad(-150), deg2rad(150), 100)),
        "Both: t1 0 to 90, t2 0 to 90": (np.linspace(0, np.pi / 2, 100),
                                          np.linspace(0, np.pi / 2, 100)),
    }
    fig, axs = plt.subplots(1, 3, figsize=(15, 5.5))
    worst = 0.0
    for ax, (title, (t1s, t2s)) in zip(axs, sweeps.items()):
        tips = np.array([fk_2link(a, b)[2] for a, b in zip(t1s, t2s)])
        paper = np.array([fk_paper(a, b) for a, b in zip(t1s, t2s)])
        worst = max(worst, float(np.abs(tips - paper).max()))

        draw_reach(ax)
        for k in range(0, 100, 20):  # a few faint arm poses along the sweep
            base, elbow, tip = fk_2link(t1s[k], t2s[k])
            ax.plot([base[0], elbow[0], tip[0]], [base[1], elbow[1], tip[1]],
                    "-o", color="gray", alpha=0.35, lw=1.5, markersize=3)
        ax.plot(tips[:, 0], tips[:, 1], color="tab:red", lw=3, label="tip (transforms)")
        ax.plot(paper[::8, 0], paper[::8, 1], "k.", markersize=7, label="paper formula")
        style(ax, title)
        ax.legend(loc="lower left", fontsize=8)
    fig.suptitle("Day 20: end-effector paths as the joints sweep")
    fig.tight_layout()
    path = os.path.join(HERE, "day20_tip_paths.png")
    fig.savefig(path, dpi=200, bbox_inches="tight")
    return path, worst


def main():
    np.set_printoptions(suppress=True)

    # Quick numeric check on the Day 19 worked example
    _, elbow, tip = fk_2link(deg2rad(30), deg2rad(60))
    print("Worked example t1=30, t2=60 -> elbow:", np.round(elbow, 4),
          " tip:", np.round(tip, 4))

    # Transforms vs paper formula on 10000 random angle pairs
    rng = np.random.default_rng(0)
    t1s = rng.uniform(-np.pi, np.pi, 10000)
    t2s = rng.uniform(-np.pi, np.pi, 10000)
    err = max(np.linalg.norm(fk_2link(a, b)[2] - fk_paper(a, b)) for a, b in zip(t1s, t2s))
    print(f"Max difference over 10000 random pairs: {err:.2e} m")

    p1 = block1_poses()
    p2, worst = block2_paths()
    print(f"Max difference along the sweeps: {worst:.2e} m")
    print("Saved", p1)
    print("Saved", p2)
    plt.show()


if __name__ == "__main__":
    main()
