"""day24_verify_ik.py: verify IK by feeding its answers back through FK."""

import os

import matplotlib.pyplot as plt
import numpy as np

from ik import UnreachableError, fk_2link, ik_2link, is_reachable, reach_limits

HERE = os.path.dirname(os.path.abspath(__file__))
L1, L2 = 2.0, 1.5
N = 100
FLOOR = 1e-17  # errors of exactly 0 are drawn at this floor on log plots


def random_reachable_targets(n, rng):
    """n random targets spread evenly over the reachable ring (by AREA)."""
    r_min, r_max = reach_limits(L1, L2)
    r = np.sqrt(rng.uniform(r_min**2, r_max**2, n))  # sqrt keeps density uniform
    a = rng.uniform(-np.pi, np.pi, n)
    return np.column_stack([r * np.cos(a), r * np.sin(a)])


def ik_fk_error(target, elbow_up):
    """Distance between the target and where FK says the IK answer lands."""
    t1, t2 = ik_2link(target[0], target[1], L1, L2, elbow_up=elbow_up)
    _, tip = fk_2link(t1, t2, L1, L2)
    return float(np.linalg.norm(tip - target))


def summarize(name, errs):
    return (f"{name:12s} max = {errs.max():.2e} m, mean = {errs.mean():.2e} m, "
            f"median = {np.median(errs):.2e} m, 95th pct = {np.percentile(errs, 95):.2e} m")


def main():
    rng = np.random.default_rng(42)
    targets = random_reachable_targets(N, rng)
    r = np.hypot(targets[:, 0], targets[:, 1])

    err_down = np.array([ik_fk_error(t, False) for t in targets])
    err_up = np.array([ik_fk_error(t, True) for t in targets])
    err_all = np.concatenate([err_down, err_up])

    print(f"Verified {N} random reachable targets, both elbow settings "
          f"({2 * N} solutions)\n")
    print(summarize("elbow-down", err_down))
    print(summarize("elbow-up", err_up))
    print(summarize("all", err_all))
    print("Worst case in millimetres:", f"{err_all.max() * 1000:.2e} mm")
    print("All errors below 1e-9 m:", bool((err_all < 1e-9).all()))

    # Unreachable targets must be rejected, never answered
    wide = rng.uniform(-5, 5, size=(1000, 2))
    rejected = 0
    wrongly_rejected = 0
    wrongly_accepted = 0
    for x, y in wide:
        reachable = is_reachable(x, y, L1, L2)
        try:
            ik_2link(x, y, L1, L2)
            if not reachable:
                wrongly_accepted += 1
        except UnreachableError:
            rejected += 1
            if reachable:
                wrongly_rejected += 1
    print(f"\n1000 random points in a 10 m x 10 m square: {rejected} rejected, "
          f"{1000 - rejected} solved")
    print(f"Wrongly rejected: {wrongly_rejected}, wrongly accepted: {wrongly_accepted}")

    # Stress test close to both limits
    r_min, r_max = reach_limits(L1, L2)
    print("\nStress test near the limits (target on the +x axis):")
    print(f"{'distance':>22s} | {'error (m)':>10s}")
    for d in [r_max - 1e-3, r_max - 1e-6, r_max - 1e-9, r_max,
              r_min + 1e-3, r_min + 1e-6, r_min + 1e-9, r_min]:
        e = ik_fk_error(np.array([d, 0.0]), False)
        print(f"{d:22.12f} | {e:10.2e}")

    # Plots
    fig, axs = plt.subplots(1, 3, figsize=(17, 5.5))

    ax = axs[0]
    ax.hist(np.log10(np.maximum(err_all, FLOOR)), bins=25, color="tab:blue", edgecolor="white")
    ax.set_xlabel("log10(error in metres)  (exact zeros drawn at -17)")
    ax.set_ylabel("Number of solutions")
    ax.set_title(f"IK -> FK error over {2 * N} solutions")
    ax.grid(True, alpha=0.3)

    ax = axs[1]
    ax.scatter(np.r_[r, r], np.maximum(err_all, FLOOR), s=18, c=["tab:blue"] * N + ["tab:red"] * N,
               alpha=0.7)
    ax.set_yscale("log")
    ax.axvline(r_min, color="k", ls=":", alpha=0.5)
    ax.axvline(r_max, color="k", ls="--", alpha=0.5)
    ax.set_xlabel("Target distance from base (m)")
    ax.set_ylabel("Error (m, log scale)")
    ax.set_title("Error vs distance (blue = elbow-down, red = elbow-up)")
    ax.grid(True, alpha=0.3)

    ax = axs[2]
    ring = np.linspace(0, 2 * np.pi, 300)
    ax.plot(r_max * np.cos(ring), r_max * np.sin(ring), "k--", alpha=0.35)
    ax.plot(r_min * np.cos(ring), r_min * np.sin(ring), "k:", alpha=0.5)
    sc = ax.scatter(targets[:, 0], targets[:, 1], c=np.log10(np.maximum(err_down, FLOOR)),
                    cmap="viridis", s=30)
    fig.colorbar(sc, ax=ax, label="log10(error) for elbow-down")
    ax.set_aspect("equal")
    ax.set_title("The 100 random targets")
    ax.grid(True, alpha=0.3)

    fig.suptitle("Day 24: verifying IK with FK")
    fig.tight_layout()
    path = os.path.join(HERE, "day24_ik_verification.png")
    fig.savefig(path, dpi=200, bbox_inches="tight")

    # A results table ready to paste into the Project 1 README
    table = (
        "| Check | Result |\n|---|---|\n"
        f"| Random reachable targets tested | {N} (x2 elbow settings) |\n"
        f"| Max IK -> FK error | {err_all.max():.2e} m |\n"
        f"| Mean IK -> FK error | {err_all.mean():.2e} m |\n"
        f"| Unreachable targets wrongly accepted | {wrongly_accepted} |\n"
        f"| Reachable targets wrongly rejected | {wrongly_rejected} |\n"
    )
    with open(os.path.join(HERE, "day24_results.md"), "w") as f:
        f.write(table)
    print("\nResults table:\n" + table)
    print("Saved", path)
    plt.show()


if __name__ == "__main__":
    main()
