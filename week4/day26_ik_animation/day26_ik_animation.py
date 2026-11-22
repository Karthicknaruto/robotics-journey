"""day26_ik_animation.py: animate the 2-link arm following a circle and a line using IK."""

import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

from ik import UnreachableError, fk_2link, ik_2link, is_reachable, reach_limits

HERE = os.path.dirname(os.path.abspath(__file__))
L1, L2 = 2.0, 1.5


# ---------- Paths: lists of (x, y) waypoints ----------
def circle_path(center, radius, n):
    """n+1 points around a circle; the last point equals the first (closed loop)."""
    t = np.linspace(0, 2 * np.pi, n + 1)
    return np.column_stack([center[0] + radius * np.cos(t),
                            center[1] + radius * np.sin(t)])


def line_path(start, end, n):
    """n points evenly spaced on a straight line."""
    return np.linspace(start, end, n)


# ---------- Solve IK for every waypoint ----------
def solve_path(path, elbow_up=False):
    """Return an (N, 2) array of joint angles in radians, one row per waypoint."""
    for i, (x, y) in enumerate(path):
        if not is_reachable(x, y, L1, L2):
            raise UnreachableError(f"Waypoint {i} at ({x:.3f}, {y:.3f}) cannot be reached")
    angles = np.array([ik_2link(x, y, L1, L2, elbow_up=elbow_up) for x, y in path])
    angles[:, 0] = np.unwrap(angles[:, 0])  # remove artificial 360-degree jumps
    return angles


def report(name, path, angles):
    tips = np.array([fk_2link(t1, t2, L1, L2)[1] for t1, t2 in angles])
    err = np.linalg.norm(tips - path, axis=1)
    step = np.abs(np.diff(np.degrees(angles), axis=0))
    print(f"\n{name}")
    print(f"  waypoints                : {len(path)}")
    print(f"  max tracking error       : {err.max():.2e} m")
    print(f"  theta1 range             : {np.degrees(angles[:, 0]).min():7.2f} to "
          f"{np.degrees(angles[:, 0]).max():7.2f} deg")
    print(f"  theta2 range             : {np.degrees(angles[:, 1]).min():7.2f} to "
          f"{np.degrees(angles[:, 1]).max():7.2f} deg")
    print(f"  biggest joint step/frame : {step.max():.2f} deg (no jumps if small)")


# ---------- Animation ----------
def animate(path, angles, title, gif_name, png_name):
    r_min, r_max = reach_limits(L1, L2)
    n = len(path)
    t = np.arange(n)
    deg = np.degrees(angles)

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(11, 5), gridspec_kw={"width_ratios": [1, 1]})

    # Left: the arm in space
    ring = np.linspace(0, 2 * np.pi, 300)
    ax.plot(r_max * np.cos(ring), r_max * np.sin(ring), "k--", alpha=0.25)
    ax.plot(r_min * np.cos(ring), r_min * np.sin(ring), "k:", alpha=0.35)
    ax.plot(path[:, 0], path[:, 1], "--", color="gray", alpha=0.7, label="desired path")
    (trail,) = ax.plot([], [], color="tab:red", lw=2.5, label="tip trail")
    (arm,) = ax.plot([], [], "-o", color="tab:blue", lw=4, markersize=7)
    (tip_dot,) = ax.plot([], [], "r*", markersize=15)
    ax.set_xlim(-r_max - 0.3, r_max + 0.3)
    ax.set_ylim(-r_max - 0.3, r_max + 0.3)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="k", lw=0.5)
    ax.axvline(0, color="k", lw=0.5)
    ax.legend(loc="lower left", fontsize=8)
    ax.set_title(title)

    # Right: the joint angles over time
    ax2.plot(t, deg[:, 0], color="tab:blue", label="theta1")
    ax2.plot(t, deg[:, 1], color="tab:orange", label="theta2")
    (dot1,) = ax2.plot([], [], "o", color="tab:blue", markersize=9)
    (dot2,) = ax2.plot([], [], "o", color="tab:orange", markersize=9)
    ax2.set_xlabel("Waypoint number")
    ax2.set_ylabel("Joint angle (deg)")
    ax2.set_title("Joint angles needed along the path")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="best")
    fig.tight_layout()

    def update(i):
        t1, t2 = angles[i]
        elbow, tip = fk_2link(t1, t2, L1, L2)
        arm.set_data([0, elbow[0], tip[0]], [0, elbow[1], tip[1]])
        tip_dot.set_data([tip[0]], [tip[1]])
        trail.set_data(path[: i + 1, 0], path[: i + 1, 1])
        dot1.set_data([i], [deg[i, 0]])
        dot2.set_data([i], [deg[i, 1]])
        return arm, tip_dot, trail, dot1, dot2

    update(n - 1)
    fig.savefig(os.path.join(HERE, png_name), dpi=150, bbox_inches="tight")  # still image for the README

    anim = FuncAnimation(fig, update, frames=n, interval=30, blit=True)
    gif_path = os.path.join(HERE, gif_name)
    anim.save(gif_path, writer=PillowWriter(fps=30), dpi=80)
    plt.close(fig)
    print(f"  saved {gif_name} ({os.path.getsize(gif_path) / 1e6:.1f} MB) and {png_name}")


def main():
    np.set_printoptions(suppress=True)

    circle = circle_path(center=(1.5, 1.0), radius=1.0, n=120)
    line = line_path(start=(-2.0, 1.0), end=(2.0, 2.0), n=120)

    for name, path, gif, png in [
        ("Circle (centre (1.5, 1.0), radius 1.0)", circle,
         "day26_circle.gif", "day26_circle.png"),
        ("Straight line (-2, 1) to (2, 2)", line,
         "day26_line.gif", "day26_line.png"),
    ]:
        angles = solve_path(path, elbow_up=False)
        report(name, path, angles)
        animate(path, angles, name, gif, png)

    # What happens if a path leaves the reachable ring?
    too_big = circle_path(center=(1.5, 1.0), radius=2.5, n=120)
    try:
        solve_path(too_big)
    except UnreachableError as err:
        print("\nCircle with radius 2.5 is rejected before animating:", err)


if __name__ == "__main__":
    main()
