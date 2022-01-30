from math import atan2, radians
from typing import Union

from numpy import linspace, max, min
from scipy.interpolate import splev, splprep

from cone import Cone
from config import *
from point import Point
from pose import Pose


def get_cones_in_front_of_car(pose: Pose, cones: list[Cone]):
    result: list[tuple[Cone, float, float]] = []
    cones_in_range = [
        Cone(c.get_index(), c.get_color(), c.get_coord())
        for c in cones
        if pose.get_coord().distance(c.get_coord()) <= RANGE_OF_VIEW
    ]

    for c in cones_in_range:
        temp = Cone(c.get_index(), c.get_color(), c.get_coord())
        temp.coord.move(-pose.get_coord().get_x(), -pose.get_coord().get_y())
        temp.coord.rotate(-pose.get_angle())
        distance = temp.coord.distance(Point(0, 0))
        angle = atan2(
            temp.get_coord().get_y(),
            temp.get_coord().get_x()
        )

        if abs(angle) <= radians(FIELD_OF_VIEW / 2.0):
            result.append((temp, distance, angle))

    return result


def interpolate_data(data):
    tck, _ = splprep(data, s=1, per=1)
    u = linspace(0, 1, num=DATA_DENSITY)

    return splev(u, tck)


def normalize_data_into_range(data, min_val, max_val):
    return (data - min(data)) / (max(data) - min(data)) * (max_val - min_val) + min_val


def select_data(data: list[Union[Cone, Pose]], sparsity: list[float]):
    result = [data[0]]
    indices = [0]
    distance = 0
    i = 1

    while i < len(data):
        if distance < sparsity[indices[-1]]:
            distance += data[i - 1].get_coord().distance(data[i].get_coord())
            i += 1
        else:
            distance_before = distance - data[i].get_coord().distance(data[i - 1].get_coord())
            sparsity[indices[-1]] = distance_before if sparsity[indices[-1]] - distance_before < distance - sparsity[indices[-1]] else distance
            result.append(data[i - 1] if sparsity[indices[-1]] - distance_before <= distance - sparsity[indices[-1]] else data[i])
            indices.append(data.index(result[-1]))
            if indices[-1] < i:
                i -= 1
            distance = 0

    return result, indices
