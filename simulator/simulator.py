import argparse
import math
from csv import reader
from random import gauss

import matplotlib.pyplot as plt
import numpy as np

from cone import Cone, ConeColor
from config import *
from odometry import Odometry
from point import Point
from pose import Pose
from utils import *


def main(directions: bool, noisy: bool, perception: bool, route: bool):
    with open('tracks/kecso.csv') as iF:
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

    # plt.plot(x, y, '.', color='red')

    dx_dt = np.gradient(x)
    dy_dt = np.gradient(y)
    d2x_dt2 = np.gradient(dx_dt)
    d2y_dt2 = np.gradient(dy_dt)

    curvature = np.abs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2) / (dx_dt * dx_dt + dy_dt * dy_dt) ** 1.5
    velocity_data = normalize_data_into_range(curvature, 0, 0.7)
    cone_distance_data = normalize_data_into_range(curvature, 0, 1 - (MIN_CONE_DISTANCE / CONE_DISTANCE))

    for i in range(len(velocity_data)):
        velocity_data[i] = (1 - velocity_data[i]) * VELOCITY

    for i in range(len(cone_distance_data)):
        cone_distance_data[i] = (1 - cone_distance_data[i]) * CONE_DISTANCE

    poses = [Pose(points[i], rotations[i]) for i in range(len(points))]
    # sparse_poses, _ = select_data(poses, (VELOCITY / 1000.0) * ODOMETRY_SAMPLING)
    velocity_data = (velocity_data / 1000.0) * ODOMETRY_SAMPLING
    sparse_poses, sparse_pose_indices = select_data(poses, velocity_data)
    velocity_data = (velocity_data / ODOMETRY_SAMPLING) * 1000.0

    odometry: list[Odometry] = []

    for i in range(len(sparse_poses) - 1):
        angle1 = sparse_poses[i + 1].get_angle()
        angle2 = sparse_poses[i].get_angle()
        if math.copysign(1, angle1) == math.copysign(1, angle2):
            difference = angle1 - angle2
        else:
            dominant = angle1 if math.pi - abs(angle1) > math.pi - abs(angle2) else angle2
            difference = abs(abs(angle1) - abs(angle2))
            math.copysign(difference, dominant)
        odometry.append(
            Odometry(
                velocity_data[sparse_pose_indices[i]],
                difference / (ODOMETRY_SAMPLING / 1000.0)
            )
        )

    odometry.append(Odometry(odometry[-1].get_speed(), odometry[-1].get_angular_velocity()))

    if route:
        coord_x = [0]
        coord_y = [0]
        phi = [0]

        for i in range(len(odometry)):
            if odometry[i].get_angular_velocity() != 0:
                R = odometry[i].get_speed() / odometry[i].get_angular_velocity()
            else:
                R = 0
            x2 = coord_x[-1] + R * (-math.sin(phi[-1]) + math.sin(phi[-1] + odometry[i].get_angular_velocity() * (ODOMETRY_SAMPLING / 1000.0)))
            y2 = coord_y[-1] + R * (math.cos(phi[-1]) - math.cos(phi[-1] + odometry[i].get_angular_velocity() * (ODOMETRY_SAMPLING / 1000.0)))
            plt.plot([coord_x[-1], x2], [coord_y[-1], y2], '-', color='green')
            coord_x.append(x2)
            coord_y.append(y2)
            phi.append(phi[-1] + odometry[i].get_angular_velocity() * (ODOMETRY_SAMPLING / 1000.0))

    x = [p.get_coord().get_x() for i, p in enumerate(sparse_poses) if i % (PERCEPTION_SAMPLING / ODOMETRY_SAMPLING) == 0]
    y = [p.get_coord().get_y() for i, p in enumerate(sparse_poses) if i % (PERCEPTION_SAMPLING / ODOMETRY_SAMPLING) == 0]

    # plt.plot(x, y, '.', color='black')

    if directions:
        for i in range(len(sparse_poses)):
            x2 = sparse_poses[i].get_coord().get_x() + 5 * math.cos(sparse_poses[i].get_angle())
            y2 = sparse_poses[i].get_coord().get_y() + 5 * math.sin(sparse_poses[i].get_angle())
            plt.plot(
                [sparse_poses[i].get_coord().get_x(), x2],
                [sparse_poses[i].get_coord().get_y(), y2],
                color='purple',
            )

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
    data = [Cone(i, ConeColor.BLUE, p) for i, p in enumerate(interpolated_left_side_points)]
    next_index = len(data)

    sparse_blue_cones, _ = select_data(data, cone_distance_data)

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
    data = [Cone(i, ConeColor.YELLOW, p) for i, p in enumerate(interpolated_right_side_points, start=next_index)]

    sparse_yellow_cones, _ = select_data(data, cone_distance_data)
    plt.plot(
        [c.get_coord().get_x() for c in sparse_yellow_cones],
        [c.get_coord().get_y() for c in sparse_yellow_cones],
        '.',
        color='orange'
    )

    timestamp = 0
    all_cones = [*sparse_blue_cones, *sparse_yellow_cones]

    actual_poses = np.zeros((len(odometry), 3))
    ts = ODOMETRY_SAMPLING / 1000.0

    for i in range(1, np.size(actual_poses, 0)):
        actual_poses[i, 0] = actual_poses[i - 1, 0] + odometry[i].get_speed() * ts * np.cos(actual_poses[i - 1, 2] + odometry[i].get_angular_velocity() * ts / 2)
        actual_poses[i, 1] = actual_poses[i - 1, 1] + odometry[i].get_speed() * ts * np.sin(actual_poses[i - 1, 2] + odometry[i].get_angular_velocity() * ts / 2)
        actual_poses[i, 2] = actual_poses[i - 1, 2] + odometry[i].get_angular_velocity() * ts

    plt.plot(actual_poses[:, 0], actual_poses[:, 1], '.', color='purple')

    with open('output.txt', 'w') as oF:
        for i in range(np.size(actual_poses, 0)):
            speed = odometry[i].get_speed()
            angular_velocity = odometry[i].get_angular_velocity()

            if noisy:
                speed += speed * gauss(0, SPEED_NOISE)
                angular_velocity += angular_velocity * gauss(0, ANGULAR_VELOCITY_NOISE)

            print('o', timestamp, speed, angular_velocity, file=oF)

            if i % (PERCEPTION_SAMPLING / ODOMETRY_SAMPLING) == 0:
                cones = get_cones_in_front_of_car(actual_poses[i], all_cones.copy())

                if perception:
                    for j in range(len(cones)):
                        x2 = actual_poses[i, 0] + cones[j][1] * math.cos(cones[j][2] + actual_poses[i, 2])
                        y2 = actual_poses[i, 1] + cones[j][1] * math.sin(cones[j][2] + actual_poses[i, 2])
                        plt.plot(
                            [actual_poses[i, 0], x2],
                            [actual_poses[i, 1], y2],
                            color='green',
                        )

                for c in cones:
                    distance = c[1]  # gauss(c[1], DISTANCE_NOISE ** 2) if noisy else c[1]
                    orientation = c[2]  # gauss(c[2], BEARING_NOISE ** 2) if noisy else c[2]
                    print('p', timestamp, distance, orientation * 180 / math.pi, c[0].get_color().value, c[0].get_index(), c[0].get_coord(), file=oF)

            timestamp += ODOMETRY_SAMPLING

    plt.axis('equal')
    plt.savefig('track.png', dpi=500)
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Perception simulator for driverless cars.', allow_abbrev=False)
    parser.add_argument('-d', '--directions', action='store_true', default=False, required=False, help='show the directional vectors of the car')
    parser.add_argument('-n', '--noisy', action='store_true', default=False, required=False, help='use noise for the output data')
    parser.add_argument('-p', '--perception', action='store_true', default=False, required=False, help='show the vectors to the cones the car is currently detecting')
    parser.add_argument('-r', '--route', action='store_true', default=False, required=False, help='show the route the car is traveling')
    args = parser.parse_args()
    main(directions=args.directions, noisy=args.noisy, perception=args.perception, route=args.route)
