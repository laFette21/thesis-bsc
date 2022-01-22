from cone import Cone, ConeColor
from point import Point
from pose import Pose


if __name__ == '__main__':
    p = Pose(Point(0, 0), 0)
    c = Cone(0, ConeColor.BLUE, Point(0, 0))
    print(p)
    print(c)
