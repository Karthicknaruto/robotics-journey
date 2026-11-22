# Day 23: Production Analytic IK Module & Workspace Exception Handling

**Date:** Tuesday (Week 4) | **Topic:** Robust Inverse Kinematics Module Architecture & Reach Boundary Enforcement

---

## 1. Objectives & Overview
- Encapsulate 2-link analytic inverse kinematics into a production-grade reusable Python module (`ik.py`).
- Implement the principle of **failing loudly**: reject unreachable Cartesian targets via custom typed exceptions (`UnreachableError`).
- Handle numerical edge cases with floating-point tolerances (`TOL = 1e-9`).
- Provide continuous fallback projection via `nearest_reachable()`.
- Validate invariants across 27 comprehensive unit tests.

---

## 2. Architectural Design Choices in `ik.py`

### Custom Exception Hierarchy
```python
class UnreachableError(ValueError):
    """Raised when a target cannot be reached by the arm."""
```
Inheriting from `ValueError` ensures compatibility with general exception catching while allowing precise identification of reachability failure.

### Strict Validation vs. Silent Failure
In safety-critical robotics, silent NaN generation or math domain exceptions can lead to erratic trajectory interpolations. Pre-checking reachability:
$$r_{\min} - \epsilon \le r \le r_{\max} + \epsilon$$
prevents invalid inputs from ever propagating into trigonometric functions ($\arccos$).

### Clamping & Slack Tolerances
Near boundaries ($r \approx r_{\max}$ or $r \approx r_{\min}$), slight floating-point imprecision can produce $|\cos\theta_2| = 1.0000000000000002$. Clamping to $[-1.0, 1.0]$ provides numerical safety against domain crashes.

### Degenerate Origin Case
When $L_1 = L_2$ and target $(x, y) = (0, 0)$, infinite solutions exist ($\theta_2 = 180^\circ$, arbitrary $\theta_1$). The solver deterministically defaults to $\theta_1 = 0^\circ$.

### Separation of Concerns: Rejection vs. Projection
- `ik_2link()` strictly enforces workspace constraints and rejects violations.
- `nearest_reachable()` computes the projection onto the closest valid workspace radius along the target line-of-sight vector:
  $$\mathbf{p}_{\text{proj}} = \hat{\mathbf{u}} \cdot \operatorname{clamp}(r, r_{\min}, r_{\max})$$

---

## 3. Unit Test Validation (27 Tests)
- **Round-Trip Consistency:** 10 parametrized test combinations of positive/negative quadrants $\times$ dual elbow branches.
- **Analytical Benchmark:** Validated against Day 22 hand-calculation ($1.8, 2.2 \implies \theta_1 = 20.54^\circ, \theta_2 = 72.24^\circ$).
- **Configuration Parity:** Verification that $\theta_2^{\text{down}} = -\theta_2^{\text{up}}$.
- **Stochastic Round-Trip:** 1,000 random reachable targets round-tripping with error $\le 10^{-8}\text{ m}$.
- **Boundary Clamping:** Fallback projection verification across inner dead-zone and exterior radii.

---

## 4. Artifacts Generated
- `day23_reach_handling.png`: Visualizing reachable target solution alongside clamped boundary projections.
- `test_ik.py`: 27 passing pytest unit tests.
