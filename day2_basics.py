import math

def deg2rad(deg: float) -> float:
    """Convert degrees to radians."""
    return deg * (math.pi / 180.0)

def rad2deg(rad: float) -> float:
    """Convert radians to degrees."""
    return rad * (180.0 / math.pi)

def point_distance(p1: tuple[float, ...], p2: tuple[float, ...] = (0.0, 0.0)) -> float:
    """Calculate Euclidean distance between two points in 2D or 3D."""
    if len(p1) != len(p2):
        raise ValueError("Both points must have identical dimensions.")
    return math.sqrt(sum((b - a) ** 2 for a, b in zip(p1, p2)))

# --- 3 Test Calls per Function ---

if __name__ == "__main__":
    print("=== Testing deg2rad ===")
    print("Test 1 (0 deg):", deg2rad(0.0))             # Expected: 0.0
    print("Test 2 (90 deg):", deg2rad(90.0))           # Expected: ~1.570796 rad (pi / 2)
    print("Test 3 (180 deg):", deg2rad(180.0))         # Expected: ~3.141592 rad (pi)

    print("\n=== Testing rad2deg ===")
    print("Test 1 (0 rad):", rad2deg(0.0))             # Expected: 0.0
    print("Test 2 (pi/2 rad):", rad2deg(math.pi / 2))  # Expected: 90.0
    print("Test 3 (pi rad):", rad2deg(math.pi))        # Expected: 180.0

    print("\n=== Testing point_distance ===")
    # 2D distance between (0, 0) and (3, 4) -> 3-4-5 right triangle
    print("Test 1 (2D standard):", point_distance((0.0, 0.0), (3.0, 4.0)))  # Expected: 5.0

    # Distance using the default origin parameter (0, 0)
    print("Test 2 (2D default origin):", point_distance((1.0, 1.0)))        # Expected: ~1.414213

    # 3D distance between (0, 0, 0) and (1, 2, 2) -> sqrt(1 + 4 + 4) = sqrt(9) = 3
    print("Test 3 (3D space):", point_distance((0.0, 0.0, 0.0), (1.0, 2.0, 2.0)))  # Expected: 3.0