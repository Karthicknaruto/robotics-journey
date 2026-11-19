"""day15_rotation_2d.py: the 2D rotation matrix, composing rotations, a rotation function."""

import numpy as np


def rot2d(theta_rad):
    """2x2 matrix that rotates a point counterclockwise by theta (radians)."""
    c, s = np.cos(theta_rad), np.sin(theta_rad)
    return np.array([[c, -s],
                     [s, c]])


def rotate_points(points, angle_deg):
    """Rotate points (2 x N, one point per column) by angle_deg about the origin."""
    return rot2d(np.radians(angle_deg)) @ points


def main():
    # 1) Where do the two axis arrows go? (this is the derivation)
    R90 = rot2d(np.radians(90))
    print("1) i-hat (1,0) after 90 deg:", np.round(R90 @ np.array([1, 0]), 10))
    print("   j-hat (0,1) after 90 deg:", np.round(R90 @ np.array([0, 1]), 10))
    print("   Columns of R are where i-hat and j-hat land:\n", np.round(R90, 10))

    # 2) Rotate a point
    p = np.array([2.0, 1.0])
    print("2) (2, 1) rotated 90 deg:", np.round(R90 @ p, 10))

    # 3) Rotating by 360 deg returns to the start
    print("3) 360 deg is identity:", np.allclose(rot2d(np.radians(360)), np.eye(2)))

    # 4) Compose: 30 deg then 60 deg equals 90 deg
    R30, R60 = rot2d(np.radians(30)), rot2d(np.radians(60))
    print("4) R60 @ R30 == R90:", np.allclose(R60 @ R30, R90))

    # 5) In 2D, rotation order does not matter
    print("5) R60 @ R30 == R30 @ R60:", np.allclose(R60 @ R30, R30 @ R60))

    # 6) Inverse of a rotation = rotate backwards = transpose
    R45 = rot2d(np.radians(45))
    print("6) R(-45) == R(45).T:", np.allclose(rot2d(np.radians(-45)), R45.T))
    print("   R @ R.T is identity:", np.allclose(R45 @ R45.T, np.eye(2)))

    # 7) Rotation keeps length and has determinant 1
    print("7) det(R):", round(float(np.linalg.det(R45)), 4))
    print("   |p| before:", round(float(np.linalg.norm(p)), 4),
          " after:", round(float(np.linalg.norm(R45 @ p)), 4))

    # 8) Rotate a whole square with the function
    square = np.array([[0, 1, 1, 0],
                       [0, 0, 1, 1]], dtype=float)
    print("8) Square rotated 90 deg:\n", np.round(rotate_points(square, 90), 10))

    # 9) Real use: a 2-link arm's elbow position
    L1 = 2.0
    elbow = rot2d(np.radians(30)) @ np.array([L1, 0.0])
    print("9) Elbow of a 2 m link at 30 deg:", np.round(elbow, 4))


if __name__ == "__main__":
    main()