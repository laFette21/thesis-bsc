from numpy import cos, sin, sqrt


class Point:
    """Create a point object."""

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def move(self, dx, dy):
        """Move the point."""
        self.x = self.x + dx
        self.y = self.y + dy

    def rotate(self, radian):
        """Rotate the point around the origin."""
        old_x, old_y = self.x, self.y
        self.x = old_x * cos(radian) - old_y * sin(radian)
        self.y = old_y * cos(radian) + old_x * sin(radian)

    def distance(self, other):
        """Calculate the distance between this and another point."""
        dx = self.x - other.x
        dy = self.y - other.y
        return sqrt(dx ** 2 + dy ** 2)

    def __str__(self):
        return f"{self.x} {self.y}"

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y
