"""Day 31: rigid-body motions. SO(3) rotations and SE(3) transforms, checked by code.

Needs only numpy.
"""
import numpy as np

TOL = 1e-9


# ---------- Building blocks ----------

def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def is_so3(R, tol=TOL):
    """True if R is a valid rotation matrix: R^T R = I and det R = +1."""
    R = np.asarray(R, dtype=float)
    if R.shape != (3, 3):
        return False
    return bool(np.allclose(R.T @ R, np.eye(3), atol=tol) and abs(np.linalg.det(R) - 1) < tol)


def random_so3(rng):
    """A random valid rotation (QR trick, then fix the sign so det = +1)."""
    q, r = np.linalg.qr(rng.normal(size=(3, 3)))
    q = q @ np.diag(np.sign(np.diag(r)))
    if np.linalg.det(q) < 0:
        q[:, 0] *= -1
    return q


def make_T(R, p):
    """4x4 homogeneous transform from a rotation R and a position p."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def is_se3(T, tol=TOL):
    """True if T is a valid 4x4 rigid transform."""
    T = np.asarray(T, dtype=float)
    return bool(T.shape == (4, 4) and is_so3(T[:3, :3], tol) and np.allclose(T[3], [0, 0, 0, 1], atol=tol))


def inv_T(T):
    """Fast inverse of a rigid transform: [R^T, -R^T p]."""
    R, p = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ p)


def transform_point(T, point):
    """Apply T to a 3D point."""
    return (T @ np.append(point, 1.0))[:3]


# ---------- Demos ----------

def main():
    rng = np.random.default_rng(0)

    print("1) Is it a rotation? (R^T R = I and det = +1)")
    print("   rot_z(30 deg)      :", is_so3(rot_z(np.radians(30))))
    print("   mirror (flip x)    :", is_so3(np.diag([-1, 1, 1])), "(det = -1, a reflection, not a rotation)")
    print("   scaled by 2        :", is_so3(2 * np.eye(3)), "(rotations never stretch)")

    print("\n2) Group rules on 1000 random rotations")
    ok_product = ok_inverse = True
    for _ in range(1000):
        A, B = random_so3(rng), random_so3(rng)
        ok_product &= is_so3(A @ B)
        ok_inverse &= np.allclose(A.T @ A, np.eye(3))
    print("   product of two rotations is a rotation:", ok_product)
    print("   inverse equals transpose              :", ok_inverse)

    print("\n3) Order matters")
    A, B = rot_x(np.radians(90)), rot_z(np.radians(90))
    print("   Rx90 @ Rz90 == Rz90 @ Rx90 ?", np.allclose(A @ B, B @ A))

    print("\n4) Fixed frame vs body frame (same two rotations, different order)")
    Rx, Rz = rot_x(np.radians(90)), rot_z(np.radians(90))
    body = Rx @ Rz   # rotate about x, then about the NEW (body) z axis
    fixed = Rz @ Rx  # rotate about x, then about the ORIGINAL (fixed) z axis
    tip = np.array([1.0, 0.0, 0.0])
    print("   point (1,0,0) after body-order  :", np.round(body @ tip, 3))
    print("   point (1,0,0) after fixed-order :", np.round(fixed @ tip, 3))

    print("\n5) Weld-gun frame chain: world -> robot base -> flange -> gun tip")
    T_world_base = make_T(rot_z(np.radians(90)), [2.0, 0.0, 0.0])   # robot stands at x = 2 m, turned 90 deg
    T_base_flange = make_T(rot_y(np.radians(90)), [0.5, 0.0, 1.0])  # flange at (0.5, 0, 1.0) m, its z-axis along the base x-axis
    T_flange_tip = make_T(np.eye(3), [0.0, 0.0, 0.3])               # gun tip 0.3 m along flange z
    T_world_tip = T_world_base @ T_base_flange @ T_flange_tip
    print("   T_world_tip =\n", np.round(T_world_tip, 3))
    print("   is it a valid SE(3)?", is_se3(T_world_tip))
    print("   tip position in the world:", np.round(T_world_tip[:3, 3], 3))

    print("\n6) Inverse: where is a weld point (in world) as seen from the gun tip?")
    weld_world = np.array([2.0, 1.0, 1.5])
    weld_in_tip = transform_point(inv_T(T_world_tip), weld_world)
    back = transform_point(T_world_tip, weld_in_tip)
    print("   weld point in gun-tip frame:", np.round(weld_in_tip, 3))
    print("   converted back to world     :", np.round(back, 3))
    print("   round-trip error            :", f"{np.linalg.norm(back - weld_world):.1e}")
    print("   T @ inv(T) == identity ?    :", np.allclose(T_world_tip @ inv_T(T_world_tip), np.eye(4)))

    print("\n7) Rigid means distances never change")
    p, q = rng.normal(size=3), rng.normal(size=3)
    T = make_T(random_so3(rng), rng.normal(size=3))
    before = np.linalg.norm(p - q)
    after = np.linalg.norm(transform_point(T, p) - transform_point(T, q))
    print(f"   distance before {before:.6f}, after {after:.6f}")


if __name__ == "__main__":
    main()