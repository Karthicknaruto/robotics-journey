"""day10_matplotlib.py: line plots, subplots, labels, saving figures."""

import os

import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def line_plot():
    """One figure, two lines, with labels, legend and grid."""
    t = np.linspace(0, 2 * np.pi, 200)
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(t, np.sin(t), label="sin(t)", color="tab:blue", linewidth=2)
    ax.plot(t, np.cos(t), label="cos(t)", color="tab:red", linestyle="--")
    ax.set_xlabel("Angle (rad)")
    ax.set_ylabel("Value")
    ax.set_title("Day 10: Line plot")
    ax.legend()
    ax.grid(True)
    path = os.path.join(HERE, "day10_line_plot.png")
    fig.savefig(path, dpi=200, bbox_inches="tight")
    return fig, path


def subplot_figure():
    """Joint angle, velocity, phase plot and torque bars in one 2x2 figure."""
    t = np.linspace(0, 4, 200)
    angle = 45 * np.sin(t)  # degrees
    velocity = np.gradient(angle, t)  # deg/s

    fig, axs = plt.subplots(2, 2, figsize=(9, 7))

    axs[0, 0].plot(t, angle)
    axs[0, 0].set_title("Joint angle")
    axs[0, 0].set_xlabel("Time (s)")
    axs[0, 0].set_ylabel("Angle (deg)")

    axs[0, 1].plot(t, velocity, color="tab:orange")
    axs[0, 1].set_title("Joint velocity")
    axs[0, 1].set_xlabel("Time (s)")
    axs[0, 1].set_ylabel("Velocity (deg/s)")

    axs[1, 0].plot(angle, velocity, color="tab:green")
    axs[1, 0].set_title("Phase plot")
    axs[1, 0].set_xlabel("Angle (deg)")
    axs[1, 0].set_ylabel("Velocity (deg/s)")

    joints = np.arange(1, 7)
    torque = np.array([12.5, 10.1, 8.4, 6.2, 4.0, 2.1])
    axs[1, 1].bar(joints, torque, color="tab:purple")
    axs[1, 1].set_title("Torque per joint")
    axs[1, 1].set_xlabel("Joint")
    axs[1, 1].set_ylabel("Torque (Nm)")

    for ax in axs.flat:
        ax.grid(True, alpha=0.3)
    fig.suptitle("Day 10: Subplots")
    fig.tight_layout()
    path = os.path.join(HERE, "day10_subplots.png")
    fig.savefig(path, dpi=200, bbox_inches="tight")
    return fig, path


def main():
    _, p1 = line_plot()
    _, p2 = subplot_figure()
    print(f"Saved {p1}")
    print(f"Saved {p2}")
    plt.show()


if __name__ == "__main__":
    main()
    