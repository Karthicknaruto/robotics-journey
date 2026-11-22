# Day 26: Continuous Cartesian Trajectory Tracking & Kinematic Animation

**Date:** Friday (Week 4) | **Topic:** Discrete Path Discretization, Branch-Consistent IK Following, and Joint Space Mapping

---

## 1. Objectives & Overview
- Solve continuous Cartesian path following (circular and linear paths) by resolving IK across dense discrete waypoints ($N = 120$).
- Maintain **configuration (branch) consistency**: prevent discontinuous flips between elbow-up and elbow-down configurations mid-motion.
- Remove phase wraps using `np.unwrap()` to ensure continuous joint profiles.
- Generate synchronized side-by-side animations (`.gif`) and still frame plots (`.png`) displaying Cartesian arm motion alongside joint space trajectories.

---

## 2. Core Kinematic Principles

### Cartesian Space vs. Joint Space Mapping
- A straight line in Cartesian space ($\mathbf{x}(t) = (1 - t)\mathbf{x}_0 + t\mathbf{x}_1$) requires non-linear, trigonometric joint motions in $(\theta_1(t), \theta_2(t))$.
- Constant end-effector linear velocity does **not** imply constant joint velocities. As the manipulator approaches folded or extended singularities, joint velocities spike significantly.

### Branch Invariance & Angle Unwrapping
- Moving from elbow-down ($\theta_2 > 0$) to elbow-up ($\theta_2 < 0$) without passing through a singularity ($\theta_2 = 0^\circ$ or $180^\circ$) would require an instantaneous infinite-velocity jump. Therefore, `elbow_up=False` is held invariant across all waypoints.
- Angular discontinuities across $\pm \pi$ boundaries are unwrapped into smooth continuous curves via `np.unwrap()`.

---

## 3. Trajectory Evaluation Results

### 1. Circular Trajectory: Center $(1.5, 1.0)$, Radius $1.0\text{ m}$ ($N = 121$ Waypoints)
- **Tracking Error:** $7.02 \times 10^{-16}\text{ m}$ (exact numerical agreement with FK).
- **$\theta_1$ Range:** $-48.33^\circ \to +19.79^\circ$.
- **$\theta_2$ Range:** $+74.48^\circ \to +159.10^\circ$ (approaching inner boundary at minimum distance $0.803\text{ m}$).
- **Peak Joint Step:** $4.00^\circ/\text{frame}$ (occurs closest to base, demonstrating kinematic non-linearity).

### 2. Linear Trajectory: $(-2.0, 1.0) \to (2.0, 2.0)$ ($N = 120$ Waypoints)
- **Tracking Error:** $8.95 \times 10^{-16}\text{ m}$.
- **$\theta_1$ Range:** $12.75^\circ \to 112.43^\circ$.
- **$\theta_2$ Range:** $73.04^\circ \to 133.53^\circ$.
- **Peak Joint Step:** $1.66^\circ/\text{frame}$.

### 3. Fail-Safe Boundary Rejection
- Circle of radius $R = 2.5\text{ m}$ evaluated: Immediately rejected at waypoint 0 ($(4.0, 1.0)$, distance $4.123\text{ m} > 3.5\text{ m}$) prior to animation execution.

---

## 4. Artifacts Generated
- `day26_circle.gif`: Animated 30 FPS playback of circular path following and joint angles.
- `day26_line.gif`: Animated 30 FPS playback of straight-line Cartesian translation.
- `day26_circle.png`: Dual-panel snapshot of complete circular trajectory.
- `day26_line.png`: Dual-panel snapshot of complete linear trajectory.
