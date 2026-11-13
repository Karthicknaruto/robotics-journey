"""day08_numpy.py: NumPy arrays, indexing, slicing, broadcasting."""

import numpy as np


def main():
    # 1) Boolean indexing: even numbers from 1 to 20
    a = np.arange(1, 21)
    print("1) Even numbers:", a[a % 2 == 0])

    # 2) Indexing and slicing on a 4x4 array
    m = np.arange(16).reshape(4, 4)
    print("2) Second column:", m[:, 1])
    print("   Last row:", m[-1, :])
    print("   Bottom-right 2x2:\n", m[2:, 2:])

    # 3) Broadcasting: subtract column means (centering)
    data = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 9.0], [7.0, 8.0, 6.0]])
    centered = data - data.mean(axis=0)
    print("3) Centered:\n", centered)
    print("   Column means now:", centered.mean(axis=0))

    # 4) Element-wise math on joint angles
    deg = np.array([0, 30, 45, 60, 90])
    rad = np.radians(deg)
    print("4) Radians:", np.round(rad, 4))
    print("   Sines:", np.round(np.sin(rad), 4))

    # 5) Identity matrix and element assignment
    i5 = np.eye(5)
    i5[2, 2] = 7
    print("5) Modified identity:\n", i5)

    # View vs copy
    x = np.array([1, 2, 3, 4])
    view = x[:2]
    view[0] = 99
    print("View changes the original:", x)
    y = np.array([1, 2, 3, 4])
    cp = y[:2].copy()
    cp[0] = 99
    print("Copy does not:", y)

    # Aggregation with axis
    t = np.array([[1, 2, 3], [4, 5, 6]])
    print("Sum axis=0:", t.sum(axis=0), "axis=1:", t.sum(axis=1))


if __name__ == "__main__":
    main()