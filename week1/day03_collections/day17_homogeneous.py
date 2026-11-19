"""day17_homogeneous.py: 4x4 homogeneous transforms (rotation plus translation)."""

import numpy as np


def rot_z(theta_rad):
    """3x3 rotation about the z-axis."""
    c, s = np.cos(theta_rad), np.sin(theta_rad)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]])


def make_transform(R, t):
    """Build a 4x4 homogeneous transform from a 3x3 rotation R and a 3-vector t."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def transform_point(T, p):
    """Apply a 4x4 transform T to a 3D point p."""
    p_h = np.append(p, 1.0)  # homogeneous form: (x, y, z, 1)
    return (T @ p_h)[:3]


def transform_direction(T, v):
    """Apply T to a direction vector (w = 0, so translation is ignored)."""
    v_h = np.append(v, 0.0)
    return (T @ v_h)[:3]


def invert_transform(T):
    """Fast inverse: [R.T, -R.T @ t] (works because R is a rotation)."""
    R, t = T[:3, :3], T[:3, 3]
    return make_transform(R.T, -R.T @ t)


def main():
    np.set_printoptions(suppress=True)

    # 1) Pure translation
    T_move = make_transform(np.eye(3), [5, 0, 0])
    print("1) Translate (1,2,3) by (5,0,0):", transform_point(T_move, [1, 2, 3]))

    # 2) Rotation plus translation (rotate first, then translate)
    T = make_transform(rot_z(np.radians(90)), [2, 0, 0])
    print("2) The 4x4 matrix:\n", np.round(T, 10))
    print("   Point (1,0,0) -> ", np.round(transform_point(T, [1, 0, 0]), 10))

    # 3) Points move with translation, directions do not
    print("3) Point (1,0,0) with w=1:", np.round(transform_point(T, [1, 0, 0]), 10))
    print("   Direction (1,0,0) with w=0:",
          np.round(transform_direction(T, [1, 0, 0]), 10))

    # 4) Compose two frames: A -> B -> C
    T_AB = make_transform(rot_z(np.radians(90)), [1, 0, 0])  # B seen from A
    T_BC = make_transform(np.eye(3), [2, 0, 0])              # C seen from B
    T_AC = T_AB @ T_BC
    print("4) Origin of C seen from A:", np.round(T_AC[:3, 3], 10))

    # 5) Order matters
    T_other = T_BC @ T_AB
    print("5) T_AB @ T_BC == T_BC @ T_AB:", np.allclose(T_AC, T_other))
    print("   Other order gives origin:", np.round(T_other[:3, 3], 10))

    # 6) Inverse undoes the transform
    T_inv = invert_transform(T)
    print("6) T @ T_inv is identity:", np.allclose(T @ T_inv, np.eye(4)))
    print("   Matches np.linalg.inv:", np.allclose(T_inv, np.linalg.inv(T)))
    p = np.array([1.0, 2.0, 3.0])
    print("   Round trip (1,2,3):", np.round(transform_point(T_inv, transform_point(T, p)), 10))

    # 7) Bottom row is always [0, 0, 0, 1]
    print("7) Bottom row of T_AC:", T_AC[3])


if __name__ == "__main__":
    main()