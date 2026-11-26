"""Day 33: MoveJ versus MoveL, shown on the 2-link arm from Project 1.

MoveJ: every joint moves in a straight line in joint space -> the tip path CURVES.
MoveL: the tip moves in a straight line in space -> the joints do whatever IK needs.

Needs the arm2link package (Project 1) in the same folder.
"""
import numpy as np
import matplotlib.pyplot as plt

from arm2link import TwoLinkArm, line_path, solve_path

ARM = TwoLinkArm(2.0, 1.5)
START = (3.0, 0.5)
END = (-1.5, 2.0)
DURATION = 2.0   # seconds for both moves
N = 100          # steps per move


def movej_angles(arm, start, end, n=N, elbow_up=False):
    """Joint angles for a MoveJ: straight-line interpolation in joint space."""
    a0 = np.array(arm.ik(*start, elbow_up=elbow_up))
    a1 = np.array(arm.ik(*end, elbow_up=elbow_up))
    t = np.linspace(0, 1, n)[:, None]
    return a0 + t * (a1 - a0)


def movel_angles(arm, start, end, n=N, elbow_up=False):
    """Joint angles for a MoveL: straight line in space, IK at every point."""
    path = line_path(start, end, n)
    return solve_path(arm, path, elbow_up=elbow_up), path


def distance_to_segment(points, a, b):
    """Distance from each point to the segment a-b."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    t = np.clip((points - a) @ d / (d @ d), 0, 1)
    nearest = a + t[:, None] * d
    return np.linalg.norm(points - nearest, axis=1)


def peak_joint_speed_deg_s(angles, duration):
    """Largest joint speed (deg/s) if the move takes `duration` seconds."""
    dt = duration / (len(angles) - 1)
    return float(np.degrees(np.abs(np.diff(angles, axis=0))).max() / dt)


def main():
    aj = movej_angles(ARM, START, END)
    al, line = movel_angles(ARM, START, END)
    tip_j = ARM.tip_positions(aj[:, 0], aj[:, 1])
    tip_l = ARM.tip_positions(al[:, 0], al[:, 1])

    dev_j = distance_to_segment(tip_j, START, END)
    dev_l = distance_to_segment(tip_l, START, END)
    length = float(np.hypot(END[0] - START[0], END[1] - START[1]))

    print(f"Move from {START} to {END} (straight distance {length:.3f} m), {DURATION} s each")
    print(f"  MoveJ  max sideways deviation : {dev_j.max() * 1000:7.1f} mm")
    print(f"  MoveL  max sideways deviation : {dev_l.max() * 1000:7.1f} mm")
    print(f"  MoveJ  peak joint speed       : {peak_joint_speed_deg_s(aj, DURATION):7.1f} deg/s")
    print(f"  MoveL  peak joint speed       : {peak_joint_speed_deg_s(al, DURATION):7.1f} deg/s")
    print(f"  MoveJ  tip speed range        : {np.linalg.norm(np.diff(tip_j, axis=0), axis=1).min() / (DURATION / (N - 1)):.2f}"
          f" to {np.linalg.norm(np.diff(tip_j, axis=0), axis=1).max() / (DURATION / (N - 1)):.2f} m/s")
    print(f"  MoveL  tip speed range        : {np.linalg.norm(np.diff(tip_l, axis=0), axis=1).min() / (DURATION / (N - 1)):.2f}"
          f" to {np.linalg.norm(np.diff(tip_l, axis=0), axis=1).max() / (DURATION / (N - 1)):.2f} m/s")

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(12, 5.2))
    ring = np.linspace(0, 2 * np.pi, 300)
    ax.plot(3.5 * np.cos(ring), 3.5 * np.sin(ring), "k--", alpha=0.25)
    ax.plot(0.5 * np.cos(ring), 0.5 * np.sin(ring), "k:", alpha=0.35)
    ax.plot(tip_j[:, 0], tip_j[:, 1], color="tab:red", lw=2.5, label="MoveJ (curved in space)")
    ax.plot(tip_l[:, 0], tip_l[:, 1], color="tab:blue", lw=2.5, ls="--", label="MoveL (straight in space)")
    ax.plot(*START, "go", markersize=10, label="start")
    ax.plot(*END, "r*", markersize=15, label="end")
    ax.set_aspect("equal")
    ax.set_xlim(-3.8, 3.8)
    ax.set_ylim(-3.8, 3.8)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower left", fontsize=8)
    ax.set_title("Tip path in the cell")

    t = np.linspace(0, DURATION, N)
    ax2.plot(t, np.degrees(aj[:, 0]), color="tab:red", label="theta1 MoveJ")
    ax2.plot(t, np.degrees(aj[:, 1]), color="tab:red", ls=":", label="theta2 MoveJ")
    ax2.plot(t, np.degrees(al[:, 0]), color="tab:blue", label="theta1 MoveL")
    ax2.plot(t, np.degrees(al[:, 1]), color="tab:blue", ls=":", label="theta2 MoveL")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Joint angle (deg)")
    ax2.set_title("Joint angles: straight lines for MoveJ, curves for MoveL")
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig("day33_movej_vs_movel.png", dpi=150, bbox_inches="tight")
    print("Saved day33_movej_vs_movel.png")


if __name__ == "__main__":
    main()