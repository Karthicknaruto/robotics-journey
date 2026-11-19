"""day13_rotating_square.py: animate a rotating square with matplotlib FuncAnimation."""

import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

HERE = os.path.dirname(os.path.abspath(__file__))


def rotation_matrix(theta_rad):
    """2x2 matrix that rotates points counterclockwise by theta."""
    c, s = np.cos(theta_rad), np.sin(theta_rad)
    return np.array([[c, -s], [s, c]])


def close(points):
    """Repeat the first corner at the end so the square closes when drawn."""
    return np.hstack([points, points[:, :1]])


def main():
    # Square corners as columns (x on top row, y on bottom row)
    square = np.array([[-1, 1, 1, -1],
                       [-1, -1, 1, 1]], dtype=float)
    i_hat = np.array([1.0, 0.0])  # where the x basis arrow starts
    j_hat = np.array([0.0, 1.0])  # where the y basis arrow starts

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(-2.5, 2.5)
    ax.set_ylim(-2.5, 2.5)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="k", lw=0.5)
    ax.axvline(0, color="k", lw=0.5)
    ax.set_title("Day 13: Rotating square")

    # Faint copy of the original square for reference
    ax.plot(*close(square), "--", color="gray", alpha=0.6, label="original")
    (line,) = ax.plot([], [], "-o", color="tab:blue", lw=2, label="rotating")
    (i_line,) = ax.plot([], [], color="tab:red", lw=3, label="i-hat")
    (j_line,) = ax.plot([], [], color="tab:green", lw=3, label="j-hat")
    angle_text = ax.text(-2.3, 2.1, "")
    ax.legend(loc="lower right")

    frames = 120  # one full turn = 120 frames

    def update(frame):
        theta = 2 * np.pi * frame / frames
        R = rotation_matrix(theta)
        pts = close(R @ square)
        line.set_data(pts[0], pts[1])
        i_new, j_new = R @ i_hat, R @ j_hat
        i_line.set_data([0, i_new[0]], [0, i_new[1]])
        j_line.set_data([0, j_new[0]], [0, j_new[1]])
        angle_text.set_text(f"angle = {np.degrees(theta):.0f} deg")
        return line, i_line, j_line, angle_text

    anim = FuncAnimation(fig, update, frames=frames, interval=30, blit=True)

    gif_path = os.path.join(HERE, "day13_rotating_square.gif")
    anim.save(gif_path, writer=PillowWriter(fps=30))
    print("Saved", gif_path)
    plt.show()


if __name__ == "__main__":
    main()
    