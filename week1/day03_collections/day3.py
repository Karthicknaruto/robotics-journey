"""day03_collections.py: lists, tuples, dicts, sets and comprehensions."""

import math


def list_stats(angles):
    """Return (largest, smallest, average) of a list of numbers."""
    return max(angles), min(angles), sum(angles) / len(angles)


def distance_from_origin(point):
    """Distance of a 3D point (x, y, z) tuple from the origin."""
    x, y, z = point  # tuple unpacking
    return math.sqrt(x**2 + y**2 + z**2)


def main():
    # 1) List: ordered, changeable collection
    angles = [10, 25, 40, 55, 70, 85, 100, 115]
    largest, smallest, average = list_stats(angles)
    print(f"1) Largest: {largest}, Smallest: {smallest}, Average: {average}")

    # 2) Tuple: ordered, unchangeable collection
    point = (3.0, 4.0, 12.0)
    print(f"2) Distance from origin: {distance_from_origin(point)}")  # 13.0

    # 3) Dict: key -> value pairs
    robot = {"name": "Arm-1", "joints": 6, "reach_m": 1.2, "payload_kg": 8}
    robot["speed_mm_s"] = 500  # add a new key
    robot["payload_kg"] = 10  # update a value
    print("3) Robot database:")
    for key, value in robot.items():
        print(f"   {key}: {value}")

    # 4) List comprehension with a condition
    values = [3, 8, 12, 5, 20, 7]
    greater = [v for v in values if v > 6]
    squares = [v**2 for v in greater]
    print(f"4) Greater than 6: {greater}, Squares: {squares}")

    # 5) Dict comprehension
    angles_rad = {j: math.radians(j * 15) for j in range(1, 7)}
    print("5) Joint -> radians:")
    for joint, rad in angles_rad.items():
        print(f"   {joint}: {rad:.4f}")

    # Bonus: set removes duplicates
    print(f"Set removes duplicates: {set([1, 2, 2, 3])}")


if __name__ == "__main__":
    main()