"""day09_linalg.py: NumPy linear algebra: dot, matmul, transpose, inverse, solve."""

import numpy as np


def main():
    # 1) Dot product of two vectors
    u = np.array([1.0, 2.0, 3.0])
    v = np.array([4.0, 5.0, 6.0])
    print("1) Dot product:", np.dot(u, v))  # 32.0

    # 2) Matrix multiplication (@ is not the same as *)
    A = np.array([[1.0, 2.0], [3.0, 4.0]])
    B = np.array([[0.0, 1.0], [1.0, 0.0]])
    print("2) A @ B:\n", A @ B)
    print("   A * B (element-wise):\n", A * B)

    # 3) Transpose
    M = np.array([[1, 2, 3], [4, 5, 6]])
    print("3) M shape:", M.shape, "-> M.T shape:", M.T.shape)
    print(M.T)

    # 4) Determinant and inverse, checked against the 2x2 hand formula
    det = np.linalg.det(A)
    inv = np.linalg.inv(A)
    print("4) det(A):", round(det, 4))  # 1*4 - 2*3 = -2
    print("   inv(A):\n", inv)
    print("   A @ inv(A):\n", np.round(A @ inv, 10))
    # Hand formula: (1/det) * [[d, -b], [-c, a]]
    a, b, c, d = A[0, 0], A[0, 1], A[1, 0], A[1, 1]
    hand = (1 / (a * d - b * c)) * np.array([[d, -b], [-c, a]])
    print("   Matches hand formula:", np.allclose(inv, hand))

    # 5) Solve Ax = b:  2x + y = 5,  x + 3y = 10
    K = np.array([[2.0, 1.0], [1.0, 3.0]])
    rhs = np.array([5.0, 10.0])
    x = np.linalg.solve(K, rhs)
    print("5) Solution [x, y]:", x)
    print("   Check K @ x:", K @ x)

    # Rotation matrix: transpose is the inverse
    theta = np.radians(30)
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta), np.cos(theta)]])
    print("R.T equals inv(R):", np.allclose(R.T, np.linalg.inv(R)))
    print("det(R):", round(np.linalg.det(R), 4))

    # Singular matrix has no inverse
    S = np.array([[1.0, 2.0], [2.0, 4.0]])
    try:
        np.linalg.inv(S)
    except np.linalg.LinAlgError as err:
        print("Singular matrix error:", err)


if __name__ == "__main__":
    main()