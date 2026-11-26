"""plotting.py: pictures and animations for the arm."""

import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

FLOOR = 1e-17  # errors of exactly 0 are drawn at this floor on log plots


def _reach_rings(ax, arm):
    ring = np.linspace(0, 2 * np.pi, 300)
    ax.plot(arm.max_reach * np.cos(ring), arm.max_reach * np.sin(ring), "k--", alpha=0.25)
    if arm.min_reach > 0.05:
        ax.plot(arm.min_reach * np.cos(ring), arm.min_reach * np.sin(ring), "k:", alpha=0.35)


def _style_axes(ax, arm, title=None):
    lim = arm.max_reach + 0.3
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="k", lw=0.5)
    ax.axvline(0, color="k", lw=0.5)
    if title:
        ax.set_title(title, fontsize=10)


def plot_poses(arm, pairs_deg, path):
    """Draw the arm for several (theta1, theta2) pairs given in degrees."""
    cols = 3
    rows = int(np.ceil(len(pairs_deg) / cols))
    fig, axs = plt.subplots(rows, cols, figsize=(4 * cols, 4 * rows))
    for ax, (a, b) in zip(np.ravel(axs), pairs_deg):
        elbow, tip = arm.fk(np.radians(a), np.radians(b))
        _reach_rings(ax, arm)
        ax.plot([0, elbow[0], tip[0]], [0, elbow[1], tip[1]], "-o", color="tab:blue", lw=3)
        ax.plot(*tip, "r*", markersize=14)
        _style_axes(ax, arm, f"t1 = {a} deg, t2 = {b} deg\ntip = ({tip[0]:.2f}, {tip[1]:.2f})")
    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_two_solutions(arm, target, path):
    """Both IK solutions (elbow-down and elbow-up) for one target."""
    fig, ax = plt.subplots(figsize=(6, 6))
    _reach_rings(ax, arm)
    for name, up, style, color in [("elbow-down", False, "-o", "tab:blue"),
                                   ("elbow-up", True, "--o", "tab:red")]:
        t1, t2 = arm.ik(*target, elbow_up=up)
        elbow, tip = arm.fk(t1, t2)
        ax.plot([0, elbow[0], tip[0]], [0, elbow[1], tip[1]], style, color=color, lw=3, label=name)
    ax.plot([0, target[0]], [0, target[1]], ":", color="gray")
    ax.plot(*target, "g*", markersize=18, label="target")
    ax.legend(loc="upper left")
    _style_axes(ax, arm, "Two IK solutions for one target")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_workspace(arm, angles, tips, path):
    """Scatter of reachable tip positions, plus how densely the samples fall by distance."""
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(12, 5.5))
    elbow_down = angles[:, 1] > 0
    ax.scatter(tips[elbow_down, 0], tips[elbow_down, 1], s=2, alpha=0.35,
               color="tab:blue", label="theta2 > 0 (elbow-down)")
    ax.scatter(tips[~elbow_down, 0], tips[~elbow_down, 1], s=2, alpha=0.35,
               color="tab:red", label="theta2 < 0 (elbow-up)")
    _reach_rings(ax, arm)
    _style_axes(ax, arm, f"Reachable workspace ({len(tips)} random joint pairs)")
    ax.legend(loc="lower left", markerscale=6, fontsize=8)

    r = np.hypot(tips[:, 0], tips[:, 1])
    ax2.hist(r, bins=40, color="tab:purple", edgecolor="white")
    ax2.axvline(arm.min_reach, color="k", ls=":", label=f"min reach {arm.min_reach} m")
    ax2.axvline(arm.max_reach, color="k", ls="--", label=f"max reach {arm.max_reach} m")
    ax2.set_xlabel("Tip distance from base (m)")
    ax2.set_ylabel("Number of samples")
    ax2.set_title("Samples bunch up near the edges")
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_verification(arm, result, path):
    """Histogram of IK -> FK errors and error versus target distance."""
    err = np.maximum(result["err_all"], FLOOR)
    n = result["n"]
    r = np.hypot(result["targets"][:, 0], result["targets"][:, 1])
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(12, 4.8))
    ax.hist(np.log10(err), bins=25, color="tab:blue", edgecolor="white")
    ax.set_xlabel("log10(error in metres)  (exact zeros drawn at -17)")
    ax.set_ylabel("Number of solutions")
    ax.set_title(f"IK -> FK error over {2 * n} solutions")
    ax.grid(True, alpha=0.3)

    ax2.scatter(np.r_[r, r], err, s=16, alpha=0.7, c=["tab:blue"] * n + ["tab:red"] * n)
    ax2.set_yscale("log")
    ax2.axvline(arm.min_reach, color="k", ls=":", alpha=0.5)
    ax2.axvline(arm.max_reach, color="k", ls="--", alpha=0.5)
    ax2.set_xlabel("Target distance from base (m)")
    ax2.set_ylabel("Error (m, log scale)")
    ax2.set_title("Error vs distance (blue = elbow-down, red = elbow-up)")
    ax2.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def animate_path(arm, path, angles, title, gif_path, png_path):
    """Animate the arm following a path; also save a still image of the last frame."""
    n = len(path)
    deg = np.degrees(angles)
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(11, 5))

    _reach_rings(ax, arm)
    ax.plot(path[:, 0], path[:, 1], "--", color="gray", alpha=0.7, label="desired path")
    (trail,) = ax.plot([], [], color="tab:red", lw=2.5, label="tip trail")
    (arm_line,) = ax.plot([], [], "-o", color="tab:blue", lw=4, markersize=7)
    (tip_dot,) = ax.plot([], [], "r*", markersize=15)
    _style_axes(ax, arm, title)
    ax.legend(loc="lower left", fontsize=8)

    t = np.arange(n)
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
        elbow, tip = arm.fk(angles[i, 0], angles[i, 1])
        arm_line.set_data([0, elbow[0], tip[0]], [0, elbow[1], tip[1]])
        tip_dot.set_data([tip[0]], [tip[1]])
        trail.set_data(path[: i + 1, 0], path[: i + 1, 1])
        dot1.set_data([i], [deg[i, 0]])
        dot2.set_data([i], [deg[i, 1]])
        return arm_line, tip_dot, trail, dot1, dot2

    update(n - 1)
    fig.savefig(png_path, dpi=150, bbox_inches="tight")
    anim = FuncAnimation(fig, update, frames=n, interval=30, blit=True)
    anim.save(gif_path, writer=PillowWriter(fps=30), dpi=80)
    plt.close(fig)
    return os.path.getsize(gif_path)
