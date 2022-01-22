from point import Point


class Pose:
    """Create a pose object."""

    def __init__(self, coord: Point, angle):
        self.coord = coord
        self.angle = angle

    def __str__(self):
        return f"{self.coord} {self.angle}"

    def get_coord(self):
        return Point(self.coord.get_x(), self.coord.get_y())

    def get_angle(self):
        return self.angle
