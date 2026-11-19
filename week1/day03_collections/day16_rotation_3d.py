"""day16_rotation_3d.py: 3D rotations about x, y, z and checks that they are valid."""

import numpy as np


def rot_x(theta_rad):
    """Rotate about the x-axis (right-hand rule)."""
    c, s = np.cos(theta_rad), np.sin(theta_rad)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]])


def rot_y(theta_rad):
    """Rotate about the y-axis (right-hand rule)."""
    c, s = np.cos(theta_rad), np.sin(theta_rad)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])


def rot_z(theta_rad):
    """Rotate about the z-axis (right-hand rule)."""
    c, s = np.cos(theta_rad), np.sin(theta_rad)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]])


def is_rotation_matrix(R, tol=1e-9):
    """True if R is orthogonal (R @ R.T = I) and has determinant +1."""
    orthogonal = np.allclose(R @ R.T, np.eye(3), atol=tol)
    det_one = np.isclose(np.linalg.det(R), 1.0, atol=tol)
    return bool(orthogonal and det_one)


def main():
    a = np.radians(90)

    # 1) Each rotation turns one axis into another
    print("1) Rz(90) on x-axis (1,0,0):", np.round(rot_z(a) @ [1, 0, 0], 10))
    print("   Rx(90) on y-axis (0,1,0):", np.round(rot_x(a) @ [0, 1, 0], 10))
    print("   Ry(90) on z-axis (0,0,1):", np.round(rot_y(a) @ [0, 0, 1], 10))

    # 2) The axis you rotate about does not move
    print("2) Rz(50) on z-axis stays:", np.round(rot_z(np.radians(50)) @ [0, 0, 1], 10))

    # 3) Check every matrix: orthogonal with determinant 1
    print("3) Validity checks (orthogonal and det = 1):")
    for name, R in [("Rx(37)", rot_x(np.radians(37))),
                    ("Ry(-120)", rot_y(np.radians(-120))),
                    ("Rz(200)", rot_z(np.radians(200)))]:
        print(f"   {name}: det = {np.linalg.det(R):.4f}, valid = {is_rotation_matrix(R)}")

    # 4) Matrices that are NOT rotations fail the check
    mirror = np.diag([1.0, 1.0, -1.0])
    stretch = np.diag([2.0, 1.0, 1.0])
    print("4) Mirror: orthogonal but det =", np.linalg.det(mirror),
          "-> valid =", is_rotation_matrix(mirror))
    print("   Stretch: det =", np.linalg.det(stretch),
          "-> valid =", is_rotation_matrix(stretch))

    # 5) Inverse = transpose
    R = rot_y(np.radians(40))
    print("5) Inverse equals transpose:", np.allclose(np.linalg.inv(R), R.T))

    # 6) In 3D the ORDER of rotations matters
    p = np.array([1.0, 0.0, 0.0])
    z_then_x = rot_x(a) @ rot_z(a) @ p  # apply Rz first, then Rx
    x_then_z = rot_z(a) @ rot_x(a) @ p  # apply Rx first, then Rz
    print("6) Rz then Rx on (1,0,0):", np.round(z_then_x, 10))
    print("   Rx then Rz on (1,0,0):", np.round(x_then_z, 10))
    print("   Same result?", np.allclose(z_then_x, x_then_z))

    # 7) Composite rotation is still a valid rotation
    combo = rot_z(np.radians(30)) @ rot_y(np.radians(45)) @ rot_x(np.radians(60))
    print("7) Composite valid rotation:", is_rotation_matrix(combo))


if __name__ == "__main__":
    main()