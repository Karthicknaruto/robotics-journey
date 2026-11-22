# Day 22: 2-Link Inverse Kinematics Theory & Derivation

**Date:** Monday (Week 4) | **Topic:** Analytical Inverse Kinematics via the Law of Cosines

---

## 1. Objectives & Overview
- Derive closed-form analytical inverse kinematics (IK) for a 2-link planar arm.
- Understand multiple kinematic branch solutions: **elbow-down** ($\theta_2 > 0$) versus **elbow-up** ($\theta_2 < 0$).
- Establish analytical reachability boundaries and singularity conditions.
- Verify both solution branches using forward kinematics (FK).

---

## 2. Step-by-Step Analytical Derivation

### Step 1: Geometric Setup
Let the base be at $(0, 0)$, elbow at $(x_1, y_1)$, and end-effector target at $(x, y)$.
The straight-line Euclidean distance from base to target is:
$$r = \sqrt{x^2 + y^2}$$

The triangle formed by base, elbow, and target has side lengths:
- $a = L_1$ (base to elbow)
- $b = L_2$ (elbow to tip)
- $c = r$ (base to tip)

The interior angle at the elbow vertex is:
$$\gamma = 180^\circ - \theta_2$$

### Step 2: Law of Cosines for $\theta_2$
By the Law of Cosines:
$$r^2 = L_1^2 + L_2^2 - 2 L_1 L_2 \cos(180^\circ - \theta_2)$$

Since $\cos(180^\circ - \theta_2) = -\cos\theta_2$:
$$r^2 = L_1^2 + L_2^2 + 2 L_1 L_2 \cos\theta_2$$

Solving for $\cos\theta_2$:
$$\cos\theta_2 = \frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}$$

### Step 3: Dual Configurations ($\pm\arccos$)
Because $\cos(-\theta) = \cos(\theta)$, there exist two solutions for $\theta_2$:
$$\theta_2 = \begin{cases} +\arccos(\cos\theta_2) & \text{(elbow-down)} \\ -\arccos(\cos\theta_2) & \text{(elbow-up)} \end{cases}$$

### Step 4: Solving for $\theta_1$
The angle of link 1 is resolved by decomposing the angle into:
1. $\alpha$: Angle from the positive $x$-axis to target $(x, y)$:
   $$\alpha = \operatorname{atan2}(y, x)$$
2. $\beta$: Angle between link 1 and the target vector line. In the coordinate frame of link 1, the tip is at $(L_1 + L_2 \cos\theta_2, L_2 \sin\theta_2)$:
   $$\beta = \operatorname{atan2}(L_2 \sin\theta_2, L_1 + L_2 \cos\theta_2)$$

Therefore:
$$\theta_1 = \alpha - \beta = \operatorname{atan2}(y, x) - \operatorname{atan2}(L_2 \sin\theta_2, L_1 + L_2 \cos\theta_2)$$

---

## 3. Boundary & Singularity Conditions
1. **Reachable Workspace Envelope:**
   $$|L_1 - L_2| \le r \le L_1 + L_2$$
   - Outer singularity ($r = L_1 + L_2$): $\cos\theta_2 = 1 \implies \theta_2 = 0^\circ$ (arm fully outstretched; solutions merge).
   - Inner singularity ($r = |L_1 - L_2|$): $\cos\theta_2 = -1 \implies \theta_2 = \pm 180^\circ$ (arm fully folded back; boundary of inner dead-zone).
2. **Out of Reach ($r > L_1 + L_2$ or $r < |L_1 - L_2|$):**
   $|\cos\theta_2| > 1$, domain of $\arccos$ is violated.

---

## 4. Worked Problem (Paper Task)
**Parameters:** $L_1 = 3.0\text{ m}, L_2 = 2.0\text{ m}$, Target $(x, y) = (3.0, 2.0)$:
1. $r^2 = 3^2 + 2^2 = 13 \implies r = 3.6056\text{ m}$.
2. Check reach: $|3 - 2| = 1 \le 3.6056 \le 3 + 2 = 5$ (Reachable $\checkmark$).
3. $\cos\theta_2 = \frac{13 - 3^2 - 2^2}{2 \times 3 \times 2} = \frac{13 - 9 - 4}{12} = 0 \implies \theta_2 = \pm 90^\circ$.
4. $\alpha = \operatorname{atan2}(2, 3) = 33.69^\circ$.
5. **Elbow-down ($\theta_2 = +90^\circ$):**
   $$\beta = \operatorname{atan2}(2 \sin 90^\circ, 3 + 2 \cos 90^\circ) = \operatorname{atan2}(2, 3) = 33.69^\circ$$
   $$\theta_1 = 33.69^\circ - 33.69^\circ = 0^\circ$$
   *Verification:* Link 1 along $x$ to $(3, 0)$, link 2 turns $90^\circ$ up to $(3, 2) \checkmark$.
6. **Elbow-up ($\theta_2 = -90^\circ$):**
   $$\beta = \operatorname{atan2}(2 \sin(-90^\circ), 3 + 2 \cos(-90^\circ)) = \operatorname{atan2}(-2, 3) = -33.69^\circ$$
   $$\theta_1 = 33.69^\circ - (-33.69^\circ) = 67.38^\circ$$

---

## 5. Artifacts Generated
- `day22_two_solutions.png`: Comparative visual plot showing both elbow-up and elbow-down configurations mirroring across the base-to-target chord.
