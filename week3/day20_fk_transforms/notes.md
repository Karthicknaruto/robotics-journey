# Day 20: 2-Link Forward Kinematics with Transforms

**Date:** Saturday (Week 3) | **Effort:** 8 hours  
**Topic:** 2-Link Planar Arm Forward Kinematics via Homogeneous Transformation Matrices

---

## 1. Objectives & Structure
- **Block 1 (4 h):** Code forward kinematics for the 2-link arm using $4 \times 4$ homogeneous transforms and plot multiple configuration angle pairs.
- **Block 2 (4 h):** Plot the end-effector trajectory (tip path) as joints sweep, verifying numerical agreement against the closed-form geometric derivation from Day 19.

---

## 2. Key Terminology & Mathematical Formulations

### Link Transform
A $4 \times 4$ matrix describing the frame at the end of a link as seen from the frame at its start:
1. Rotate by joint angle $\theta$ about the $z$-axis.
2. Translate by link length $L$ along the new $x$-axis.

$$
T(\theta, L) = \begin{bmatrix}
\cos\theta & -\sin\theta & 0 & L\cos\theta \\
\sin\theta & \cos\theta & 0 & L\sin\theta \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

The fourth column represents the position vector of the subsequent joint relative to the current frame:
$$
\mathbf{p} = \begin{bmatrix} L\cos\theta \\ L\sin\theta \\ 0 \\ 1 \end{bmatrix}
$$

### Chaining Transforms
By the transformation composition rule ($T_{AC} = T_{AB} T_{BC}$):
$$
T_{\text{tip}} = T_1(\theta_1, L_1) \cdot T_2(\theta_2, L_2)
$$
- $T_1$: Frame at elbow relative to base.
- $T_2$: Frame at tip relative to elbow.
- $T_{\text{tip}}$: Frame at tip relative to base.
The second link's absolute angle naturally resolves as $\theta_1 + \theta_2$ due to matrix rotation composition without manual angular accumulation.

### Workspace Boundaries (Reach Circles)
For link lengths $L_1 = 2.0\text{ m}, L_2 = 1.5\text{ m}$:
- **Outer boundary (Maximum reach):** $r_{\max} = L_1 + L_2 = 3.5\text{ m}$ (arm fully extended, $\theta_2 = 0^\circ$).
- **Inner boundary (Dead zone):** $r_{\min} = |L_1 - L_2| = 0.5\text{ m}$ (arm fully folded back, $\theta_2 = \pm 180^\circ$).

---

## 3. Worked Example

Given $L_1 = 2.0\text{ m}, L_2 = 1.5\text{ m}, \theta_1 = 30^\circ, \theta_2 = 60^\circ$:

1. **Elbow Position:**
   $$x_1 = 2.0 \cos(30^\circ) = 1.7321\text{ m},\quad y_1 = 2.0 \sin(30^\circ) = 1.0\text{ m}$$
2. **Tip Position in Link 1 Frame:**
   $$x_{2/1} = 1.5 \cos(60^\circ) = 0.75\text{ m},\quad y_{2/1} = 1.5 \sin(60^\circ) = 1.2990\text{ m}$$
3. **Tip Position in Base Frame:**
   $$
   \begin{bmatrix} x_{\text{tip}} \\ y_{\text{tip}} \end{bmatrix} = \begin{bmatrix} \cos 30^\circ & -\sin 30^\circ \\ \sin 30^\circ & \cos 30^\circ \end{bmatrix} \begin{bmatrix} 0.75 \\ 1.2990 \end{bmatrix} + \begin{bmatrix} 1.7321 \\ 1.0 \end{bmatrix} = \begin{bmatrix} 1.7321 \\ 2.5000 \end{bmatrix}
   $$

Numeric verification confirms exact equivalence with closed-form paper derivation:
$$\Delta = |T_{\text{tip}}[:2, 3] - \mathbf{p}_{\text{paper}}| \approx 1.11 \times 10^{-15}\text{ m}$$

---

## 4. Observations from Joint Sweeps
1. **Sweeping $\theta_1$ ($\theta_2 = 60^\circ$ fixed):** Tip traces an arc of constant radius $r = \sqrt{L_1^2 + L_2^2 + 2L_1L_2\cos 60^\circ} = 3.0414\text{ m}$ centered at base origin. Reach depends strictly on $\theta_2$.
2. **Sweeping $\theta_2$ ($\theta_1 = 30^\circ$ fixed):** Tip traces a circle of radius $L_2 = 1.5\text{ m}$ centered at the elbow $(1.732, 1.0)$.
3. **Simultaneous sweep:** Generates non-linear Cartesian curve trajectories, laying the empirical foundation for Inverse Kinematics.

---

## 5. Artifacts Generated
- `day20_arm_poses.png`: Arm configurations across 6 distinct angle pairs.
- `day20_tip_paths.png`: Trajectory plots comparing transform chain results against the closed-form formulation.
