from enum import Enum

from point import Point


class ConeColor(Enum):
    """Enum for cone color."""

    BLUE = 1
    YELLOW = 2


class Cone:
    """Create a cone object."""

    def __init__(self, index: int, color: ConeColor, coord: Point):
        self.index = index
        self.color = color
        self.coord = coord

    def __str__(self) -> str:
        return ' '.join([
            f"{self.coord}",
            f"{self.color.value}",
            f"{self.index}"
        ])

    def get_index(self) -> int:
        return self.index

    def get_color(self) -> ConeColor:
        return self.color

    def get_coord(self) -> Point:
        return Point(self.coord.get_x(), self.coord.get_y())
