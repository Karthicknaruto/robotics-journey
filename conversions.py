# conversions.py
import math

def deg2rad(deg: float) -> float:
    """Convert degrees to radians."""
    return deg * (math.pi / 180.0)

def rad2deg(rad: float) -> float:
    """Convert radians to degrees."""
    return rad * (180.0 / math.pi)