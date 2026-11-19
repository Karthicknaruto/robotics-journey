import math

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance_to(self, other):
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def __repr__(self):
        return f"Point({self.x}, {self.y})"


class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def magnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def __repr__(self):
        return f"Vector({self.x}, {self.y})"


p1 = Point(0, 0)
p2 = Point(3, 4)
print(p1.distance_to(p2))      # 5.0

v = Vector(3, 4)
print(v.magnitude())           # 5.0
print(v.dot(Vector(1, 0)))     # 3
