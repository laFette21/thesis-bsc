import math
from csv import reader

import matplotlib.pyplot as plt
import numpy as np

from cone import Cone, ConeColor
from odometry import Odometry
from point import Point
from pose import Pose
from utils import *


def main():
    with open('tracks/FSG21.csv') as iF:
        csv_reader = reader(iF)
        x = []
        y = []
        for row in csv_reader:
            x.append(float(row[0]))
            y.append(float(row[1]))
        x.append(x[0])
        y.append(y[0])

    track = interpolate_data([x, y])
    points = [Point(track[0][i], track[1][i]) for i in range(len(track[0]))]

    # Modifying the coordinate system
    origin = Point(points[0].get_x(), points[0].get_y())
    first_rotation = math.atan2(points[1].get_y() - origin.get_y(), points[1].get_x() - origin.get_x())

    for p in points:
        p.move(-origin.get_x(), -origin.get_y())
        p.rotate(-first_rotation)

    # Calculating the angle of rotation for odometry
    rotations = [0]

    for i in range(1, len(points) - 1):
        angle1 = math.atan2(points[i].get_y() - points[i - 1].get_y(), points[i].get_x() - points[i - 1].get_x())
        angle2 = math.atan2(points[i + 1].get_y() - points[i].get_y(), points[i + 1].get_x() - points[i].get_x())
        if math.copysign(1, angle1) == math.copysign(1, angle2):
            angle = angle1 + angle2
        else:
            dominant = angle1 if math.pi - abs(angle1) > math.pi - abs(angle2) else angle2
            angle = abs(angle1) + abs(angle2)
            math.copysign(angle, dominant)
        rotations.append(angle / 2.0)

    rotations.append(rotations[-1])

    x = [p.get_x() for p in points]
    y = [p.get_y() for p in points]

    plt.plot(x, y, '.', color='red')

    dx_dt = np.gradient(x)
    dy_dt = np.gradient(y)
    d2x_dt2 = np.gradient(dx_dt)
    d2y_dt2 = np.gradient(dy_dt)

    curvature = np.abs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2) / (dx_dt * dx_dt + dy_dt * dy_dt) ** 1.5
    velocity_data = normalize_data_into_range(curvature, 0.3, 1.0) * VELOCITY
    cone_distance_data = normalize_data_into_range(curvature, 0.7, 1.0) * CONE_DISTANCE

    poses = [Pose(points[i], rotations[i]) for i in range(len(points))]
    sparse_poses, _ = select_data(poses, (VELOCITY / 1000.0) * ODOMETRY_SAMPLING)

    odometry: list[Odometry] = []

    for i in range(len(sparse_poses) - 1):
        odometry.append(
            Odometry(VELOCITY, (sparse_poses[i + 1].get_angle() - sparse_poses[i].get_angle()) / ODOMETRY_SAMPLING)
        )

    odometry.append(Odometry(odometry[-1].get_speed(), odometry[-1].get_angular_velocity()))

    x = [p.get_coord().get_x() for i, p in enumerate(sparse_poses) if i % (PERCEPTION_SAMPLING / ODOMETRY_SAMPLING) == 0]
    y = [p.get_coord().get_y() for i, p in enumerate(sparse_poses) if i % (PERCEPTION_SAMPLING / ODOMETRY_SAMPLING) == 0]

    plt.plot(x, y, '.', color='black')
    """
    for i in range(len(sparse_poses)):
        # print(sparse_poses[i].get_angle() * 180 / math.pi)
        x2 = sparse_poses[i].get_coord().get_x() + 5 * math.cos(sparse_poses[i].get_angle())
        y2 = sparse_poses[i].get_coord().get_y() + 5 * math.sin(sparse_poses[i].get_angle())
        # plt.plot(
        #     [sparse_poses[i].get_coord().get_x(), x2],
        #     [sparse_poses[i].get_coord().get_y(), y2],
        #     color='purple',
        # )
    """
    gradient_x = np.gradient(x)
    gradient_y = np.gradient(y)

    left_side_points: list[Point] = []
    right_side_points: list[Point] = []

    for i in range(len(x)):
        gradient_norm = math.sqrt(gradient_x[i] * gradient_x[i] + gradient_y[i] * gradient_y[i])
        left_side_points.append(Point(
            x[i] + TRACK_HALF_WIDTH * (-gradient_y[i] / gradient_norm),
            y[i] + TRACK_HALF_WIDTH * (gradient_x[i] / gradient_norm)
        ))
        right_side_points.append(Point(
            x[i] - TRACK_HALF_WIDTH * (-gradient_y[i] / gradient_norm),
            y[i] - TRACK_HALF_WIDTH * (gradient_x[i] / gradient_norm)
        ))

    data = [
        [p.get_x() for p in left_side_points],
        [p.get_y() for p in left_side_points]
    ]
    data[0].append(left_side_points[0].get_x())
    data[1].append(left_side_points[0].get_y())

    track = interpolate_data(data)
    interpolated_left_side_points = [Point(track[0][i], track[1][i]) for i in range(len(track[0]))]
    # plt.plot(track[0], track[1], '.', color='blue')
    data = [Cone(i, ConeColor.BLUE, p) for i, p in enumerate(interpolated_left_side_points)]

    sparse_blue_cones, _ = select_data(data, CONE_DISTANCE)
    plt.plot(
        [c.get_coord().get_x() for c in sparse_blue_cones],
        [c.get_coord().get_y() for c in sparse_blue_cones],
        '.',
        color='blue'
    )

    data = [
        [p.get_x() for p in right_side_points],
        [p.get_y() for p in right_side_points]
    ]
    data[0].append(right_side_points[0].get_x())
    data[1].append(right_side_points[0].get_y())

    track = interpolate_data(data)

    interpolated_right_side_points = [Point(track[0][i], track[1][i]) for i in range(len(track[0]))]
    # plt.plot(track[0], track[1], '.', color='orange')
    data = [Cone(i, ConeColor.YELLOW, p) for i, p in enumerate(interpolated_right_side_points)]

    sparse_yellow_cones, _ = select_data(data, CONE_DISTANCE)
    plt.plot(
        [c.get_coord().get_x() for c in sparse_yellow_cones],
        [c.get_coord().get_y() for c in sparse_yellow_cones],
        '.',
        color='orange'
    )

    timestamp = 0
    all_cones = [*sparse_blue_cones, *sparse_yellow_cones]

    with open('output.txt', 'w') as oF:
        print(ODOMETRY_SAMPLING, PERCEPTION_SAMPLING, file=oF)
        print(file=oF)

        for i in range(len(sparse_poses)):
            print('o', timestamp, odometry[i], file=oF)

            if i % (PERCEPTION_SAMPLING / ODOMETRY_SAMPLING) == 0:
                cones = get_cones_in_front_of_car(sparse_poses[i], all_cones.copy())

                for j in range(len(cones)):
                    x2 = sparse_poses[i].get_coord().get_x() + cones[j][1] * math.cos(cones[j][2] + sparse_poses[i].get_angle())
                    y2 = sparse_poses[i].get_coord().get_y() + cones[j][1] * math.sin(cones[j][2] + sparse_poses[i].get_angle())
                    plt.plot(
                        [sparse_poses[i].get_coord().get_x(), x2],
                        [sparse_poses[i].get_coord().get_y(), y2],
                        color='purple',
                    )

                for c in cones:
                    print('p', timestamp, c[1], c[2] * 180 / math.pi, c[0].get_color().value, c[0].get_index(), file=oF)

            timestamp += ODOMETRY_SAMPLING

    plt.axis('equal')
    plt.savefig('track.png', dpi=500)
    plt.show()


if __name__ == '__main__':
    main()
