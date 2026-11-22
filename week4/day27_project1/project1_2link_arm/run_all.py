"""run_all.py: regenerate every result for Project 1 (2-link arm).

Run from the project root:  python run_all.py
"""

import os

import numpy as np

from arm2link import (TwoLinkArm, circle_path, joint_steps_deg, line_path,
                      sample_workspace, solve_path, tracking_error, verify_ik,
                      workspace_summary)
from arm2link.plotting import (animate_path, plot_poses, plot_two_solutions,
                               plot_verification, plot_workspace)

OUT = "results"


def main():
    os.makedirs(OUT, exist_ok=True)
    arm = TwoLinkArm(2.0, 1.5)
    print(f"Arm: L1={arm.L1} m, L2={arm.L2} m, reach {arm.min_reach}..{arm.max_reach} m")

    # 1. IK verification
    res = verify_ik(arm, n=100, seed=42, n_wide=1000)
    plot_verification(arm, res, f"{OUT}/verification.png")
    print(f"IK check: max error {res['max_error']:.2e} m, mean {res['mean_error']:.2e} m")
    print(f"Wide test: {res['wide_solved']} solved, {res['n_wide'] - res['wide_solved']} rejected, "
          f"{res['wrongly_accepted']} wrongly accepted, {res['wrongly_rejected']} wrongly rejected")

    # 2. Workspace
    angles, tips = sample_workspace(arm, n=20000, seed=0)
    ws = workspace_summary(arm, tips)
    plot_workspace(arm, angles, tips, f"{OUT}/workspace.png")
    print(f"Workspace: observed reach {ws['observed_min_reach']:.3f}..{ws['observed_max_reach']:.3f} m, "
          f"area {ws['area_m2']:.2f} m^2")

    # 3. Poses and two solutions
    plot_poses(arm, [(0, 0), (45, 90), (90, -90), (30, 60), (-45, 120), (120, -45)], f"{OUT}/poses.png")
    plot_two_solutions(arm, (2.0, 2.0), f"{OUT}/two_solutions.png")

    # 4. Path following
    circle = circle_path((1.5, 1.0), 1.0, 120)
    line = line_path((-2.0, 1.0), (2.0, 2.0), 120)
    rows = []
    for name, path in [("circle", circle), ("line", line)]:
        a = solve_path(arm, path)
        err = tracking_error(arm, path, a).max()
        step = joint_steps_deg(a).max()
        size = animate_path(arm, path, a, f"{name} path", f"{OUT}/{name}.gif", f"{OUT}/{name}.png")
        rows.append((name, err, step, np.degrees(a[:, 1]).max(), size / 1e6))
        print(f"{name}: max tracking error {err:.2e} m, biggest joint step {step:.2f} deg/frame, "
              f"GIF {size / 1e6:.2f} MB")

    with open(f"{OUT}/summary.md", "w") as f:
        f.write("# Results summary\n\n")
        f.write(f"- Arm: L1 = {arm.L1} m, L2 = {arm.L2} m\n")
        f.write(f"- IK max error: {res['max_error']:.2e} m; mean: {res['mean_error']:.2e} m\n")
        f.write(f"- Wide test ({res['n_wide']} points): {res['wide_solved']} solved, "
                f"{res['wrongly_accepted']} wrongly accepted, {res['wrongly_rejected']} wrongly rejected\n")
        f.write(f"- Workspace area: {ws['area_m2']:.2f} m^2\n")
        for name, err, step, t2max, size in rows:
            f.write(f"- {name}: max error {err:.2e} m, biggest joint step {step:.2f} deg\n")
    print("Saved everything in", OUT)


if __name__ == "__main__":
    main()
