from typing import Union

import matplotlib.pyplot as plt
from numpy import arctan2, cos, linspace, max, min, pi, radians, sin, size
from numpy.random import normal
from scipy.interpolate import splev, splprep

from cone import Cone
from config import *
from point import Point
from pose import Pose


def get_cones_in_front_of_car(pose, cones: list[Cone]):
    result: list[tuple[Cone, float, float]] = []
    pose = Pose(Point(pose[0], pose[1]), pose[2])
    cones_in_range = [
        Cone(c.get_index(), c.get_color(), c.get_coord())
        for c in cones
        if pose.get_coord().distance(c.get_coord()) <= RANGE_OF_VIEW
    ]

    for c in cones_in_range:
        original_position = c.get_coord()
        temp = Cone(c.get_index(), c.get_color(), c.get_coord())
        temp.coord.move(-pose.get_coord().get_x(), -pose.get_coord().get_y())
        temp.coord.rotate(-pose.get_orientation())
        distance = temp.coord.distance(Point(0, 0))
        angle = arctan2(
            temp.get_coord().get_y(),
            temp.get_coord().get_x()
        )
        temp.coord = original_position

        if abs(angle) <= (FIELD_OF_VIEW / 2.0):
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
            if data[i - 1] not in result:
                sparsity[indices[-1]] = distance_before if sparsity[indices[-1]] - distance_before < distance - sparsity[indices[-1]] else distance
                result.append(data[i - 1] if sparsity[indices[-1]] - distance_before <= distance - sparsity[indices[-1]] else data[i])
            else:
                sparsity[indices[-1]] = distance
                result.append(data[i])
            indices.append(data.index(result[-1]))
            if indices[-1] < i:
                i -= 1
            distance = 0

    return result, indices


def write_data_to_file(filename, cones, poses, motion, noisy, perception):
    timestamp = 0

    with open(filename, 'w') as oF:
        for i in range(size(poses, 0)):
            speed = motion[i].get_speed()
            angular_velocity = motion[i].get_angular_velocity()

            if noisy:
                speed += speed * normal(0, SPEED_NOISE)
                angular_velocity += angular_velocity * normal(0, ANGULAR_VELOCITY_NOISE)

            print('o', round(timestamp, 3), speed, angular_velocity, file=oF)

            if i % int(PERCEPTION_SAMPLING / MOTION_SAMPLING) == 0:
                cones_in_range = get_cones_in_front_of_car(poses[i], cones)

                if perception:
                    for j in range(len(cones_in_range)):
                        x2 = poses[i, 0] + cones_in_range[j][1] * cos(cones_in_range[j][2] + poses[i, 2])
                        y2 = poses[i, 1] + cones_in_range[j][1] * sin(cones_in_range[j][2] + poses[i, 2])
                        plt.plot(
                            [poses[i, 0], x2],
                            [poses[i, 1], y2],
                            color='red',
                        )

                for c in cones_in_range:
                    distance = c[1]
                    orientation = c[2]
                    print('p', round(timestamp, 3), distance, orientation, c[0].get_color().value, c[0].get_index(), c[0].get_coord(), file=oF)

            timestamp += MOTION_SAMPLING
