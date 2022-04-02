from point import Point


class Pose:
    """Create a pose object."""

    def __init__(self, coord: Point, orientation: float):
        self.coord = coord
        self.orientation = orientation

    def __str__(self) -> str:
        return f"{self.coord} {self.orientation}"

    def get_coord(self) -> Point:
        return Point(self.coord.get_x(), self.coord.get_y())

    def get_orientation(self) -> float:
        return self.orientation
