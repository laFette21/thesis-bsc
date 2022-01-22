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


def select_data(data: list[Union[Cone, Pose]], sparsity):
    result = [data[0]]
    indices = [0]
    distance = 0
    curr = 0
    i = 1

    while i < len(data):
        if distance + data[curr].get_coord().distance(data[i].get_coord()) <= sparsity:
            distance += data[curr].get_coord().distance(data[i].get_coord())
            curr = i
            i += 1
        else:
            if distance < sparsity:
                difference_after = distance + data[curr].get_coord().distance(data[curr + 1].get_coord()) - sparsity
                difference_before = sparsity - distance
                temp = data[curr + 1] if difference_after <= difference_before else data[curr]
                curr += 1 if difference_after <= difference_before else 0
                i += 0 if difference_after <= difference_before else 1
            else:
                temp = data[curr]
            result.append(temp)
            indices.append(data.index(temp))
            distance = 0

    return result, indices
