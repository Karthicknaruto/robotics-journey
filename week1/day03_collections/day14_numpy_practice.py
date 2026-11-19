"""day14_numpy_practice.py: 15 NumPy practice problems with self-checking answers."""

import numpy as np


def main():
    results = []

    def check(n, description, value, expected):
        ok = np.allclose(value, expected)
        results.append(ok)
        print(f"{n:>2}) {'PASS' if ok else 'FAIL'}  {description}: {value}")

    # 1. Numbers 0 to 9
    check(1, "arange 0..9", np.arange(10), [0, 1, 2, 3, 4, 5, 6, 7, 8, 9])

    # 2. Reshape 1..12 into 3 rows x 4 columns (check the shape)
    m = np.arange(1, 13).reshape(3, 4)
    check(2, "shape of reshaped array", m.shape, (3, 4))

    # 3. 3x3 identity matrix
    check(3, "identity 3x3 diagonal sum", np.eye(3).sum(), 3.0)

    # 4. Even numbers from 1..20
    a = np.arange(1, 21)
    check(4, "evens", a[a % 2 == 0], [2, 4, 6, 8, 10, 12, 14, 16, 18, 20])

    # 5. Replace negatives with 0
    b = np.array([-3, 5, -1, 8])
    b[b < 0] = 0
    check(5, "negatives to 0", b, [0, 5, 0, 8])

    # 6. Column means
    c = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]])
    check(6, "column means", c.mean(axis=0), [4.0, 5.0, 6.0])

    # 7. Row sums
    check(7, "row sums", c.sum(axis=1), [6.0, 15.0, 24.0])

    # 8. Unit vector of (3, 4)
    v = np.array([3.0, 4.0])
    check(8, "unit vector", v / np.linalg.norm(v), [0.6, 0.8])

    # 9. Dot product
    check(9, "dot product", np.dot([1, 2, 3], [4, 5, 6]), 32)

    # 10. Matrix multiplication
    A = np.array([[1, 2], [3, 4]])
    B = np.array([[0, 1], [1, 0]])
    check(10, "A @ B", A @ B, [[2, 1], [4, 3]])

    # 11. Solve 2x + y = 5, x + 3y = 10
    sol = np.linalg.solve(np.array([[2.0, 1.0], [1.0, 3.0]]), np.array([5.0, 10.0]))
    check(11, "solve system", sol, [1.0, 3.0])

    # 12. Five evenly spaced numbers from 0 to 1
    check(12, "linspace", np.linspace(0, 1, 5), [0, 0.25, 0.5, 0.75, 1.0])

    # 13. Reverse an array
    check(13, "reversed", np.array([1, 2, 3, 4])[::-1], [4, 3, 2, 1])

    # 14. Distance between two 3D points
    p, q = np.array([1.0, 2.0, 3.0]), np.array([4.0, 6.0, 15.0])
    check(14, "3D distance", np.linalg.norm(q - p), 13.0)

    # 15. Position of the largest value in a 2D array
    d = np.array([[3, 9, 2], [7, 1, 8]])
    row, col = np.unravel_index(np.argmax(d), d.shape)
    check(15, "argmax (row, col)", (int(row), int(col)), (0, 1))

    print(f"\nScore: {sum(results)} / {len(results)}")


if __name__ == "__main__":
    main()
    