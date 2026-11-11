import math

class Robot:
    def __init__(self, name, x=0.0, y=0.0, heading_deg=0.0):
        self.name = name
        self.x = x
        self.y = y
        self.heading_deg = heading_deg
        self.path = [(x, y)]            # history of positions

    def move(self, distance):
        """Move forward in the current heading."""
        rad = math.radians(self.heading_deg)
        self.x += distance * math.cos(rad)
        self.y += distance * math.sin(rad)
        self.path.append((self.x, self.y))

    def turn(self, angle_deg):
        """Rotate by angle_deg (positive = counterclockwise)."""
        self.heading_deg = (self.heading_deg + angle_deg) % 360

    def distance_travelled(self):
        total = 0.0
        for (x1, y1), (x2, y2) in zip(self.path, self.path[1:]):
            total += math.hypot(x2 - x1, y2 - y1)
        return total

    def __repr__(self):
        return (f"Robot({self.name}, x={self.x:.2f}, y={self.y:.2f}, "
                f"heading={self.heading_deg:.0f})")


if __name__ == "__main__":
    r = Robot("Bot-1")
    r.move(5)
    r.turn(90)
    r.move(3)
    print(r)
    print("Distance travelled:", r.distance_travelled())