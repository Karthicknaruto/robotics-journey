# Week 3 Review

## What I finished
- [x] 2D rotation matrix (derivation, composition, function)
- [x] 3D rotations about x, y, z (orthogonal, determinant 1)
- [x] Homogeneous 4x4 transforms
- [x] Frames: expressing a point in another frame, transform() helper
- [x] 2-link forward kinematics on paper
- [x] FK with transforms, pose plots and sweep paths
- [x] N-link arm with a loop, pytest tests

## Hardest topic and why
- Chaining transforms in 3D and ensuring the coordinate frame transformations follow the correct multiplication order ($T_{AC} = T_{AB} \cdot T_{BC}$).

## 3 things still unclear
1. How joint limits are mathematically incorporated into analytic solutions vs numerical solvers.
2. The exact transition from 2D planar link transforms to general 3D Denavit-Hartenberg (DH) parameter convention.
3. Singularity handling when multiple configurations collapse into a single manifold.

## Self-rating (1 to 5)
| Topic | Rating |
|---|---|
| 2D and 3D rotation matrices | 5 |
| Homogeneous transforms | 5 |
| Frames and inverse transforms | 5 |
| 2-link FK derivation | 5 |
| Chaining transforms in a loop | 5 |
| Writing pytest tests | 5 |

## Can I do these from memory (yes / no)?
- Write the 2-link tip formula: Yes ($x = L_1 \cos\theta_1 + L_2 \cos(\theta_1 + \theta_2)$, $y = L_1 \sin\theta_1 + L_2 \sin(\theta_1 + \theta_2)$)
- Explain why joint order matters in 3D: Yes (3D rotations are non-commutative; $R_x R_y \neq R_y R_x$)
- Convert a point from frame A to frame B: Yes ($P_B = T_{BA} P_A = T_{AB}^{-1} P_A$)

## Hours studied vs plan
- Planned: 16 hours (Day 20: 8h, Day 21: 8h). Completed: 16 hours.

## Plan for Week 4 (inverse kinematics)
- Derive and code 2-link analytic inverse kinematics using the Law of Cosines.
- Handle elbow-up and elbow-down configurations.
- Formulate reachability criteria and dead-zone rejection.
- Run large-scale verification against FK and generate workspace plots.
- Animate Cartesian path tracking (circle, linear moves) via IK.
