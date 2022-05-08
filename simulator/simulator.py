import argparse
import os
from csv import reader

import matplotlib.pyplot as plt
import numpy as np

from cone import Cone, ConeColor
from config import *
from motion import Motion
from point import Point
from pose import Pose
from utils import *


def main(path: str, directions: bool, noisy: bool, perception: bool):
    # Reading the csv file
    with open(path) as iF:
        _, filename = os.path.split(path)
        filename = filename.split('.')[0]
        csv_reader = reader(iF)
        x = []
        y = []
        for row in csv_reader:
            x.append(float(row[0]))
            y.append(float(row[1]))
        x.append(x[0])
        y.append(y[0])

    # Interpolating the read points to be more dense
    track = interpolate_data([x, y])
    points = [Point(track[0][i], track[1][i]) for i in range(len(track[0]))]

    # Modifying the coordinate system
    origin = Point(points[0].get_x(), points[0].get_y())
    first_rotation = np.arctan2(points[1].get_y() - origin.get_y(), points[1].get_x() - origin.get_x())

    for p in points:
        p.move(-origin.get_x(), -origin.get_y())
        p.rotate(-first_rotation)

    # Calculating the orientation for poses
    orientation = [0]

    for i in range(1, len(points) - 1):
        angle1 = np.arctan2(points[i].get_y() - points[i - 1].get_y(), points[i].get_x() - points[i - 1].get_x())
        angle2 = np.arctan2(points[i + 1].get_y() - points[i].get_y(), points[i + 1].get_x() - points[i].get_x())
        if np.copysign(1, angle1) == np.copysign(1, angle2):
            angle = angle1 + angle2
        else:
            dominant = angle1 if np.pi - abs(angle1) > np.pi - abs(angle2) else angle2
            angle = abs(angle1) + abs(angle2)
            np.copysign(angle, dominant)
        orientation.append(angle / 2.0)

    orientation.append(orientation[-1])

    x = [p.get_x() for p in points]
    y = [p.get_y() for p in points]

    # plt.plot(x, y, '.', color='red')

    # Calculating the first and second derivatives of x and y
    dx_dt = np.gradient(x)
    dy_dt = np.gradient(y)
    d2x_dt2 = np.gradient(dx_dt)
    d2y_dt2 = np.gradient(dy_dt)

    # Calculating the curvature
    curvature = np.abs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2) / (dx_dt * dx_dt + dy_dt * dy_dt) ** 1.5

    # Normalizing the curvature data
    velocity_data = normalize_data_into_range(curvature, 0, 0.7)
    cone_distance_data = normalize_data_into_range(curvature, 0, 1 - (MIN_CONE_DISTANCE / CONE_DISTANCE))

    # Using the normalized curvature data to calculate velocity
    for i in range(len(velocity_data)):
        velocity_data[i] = (1 - velocity_data[i]) * VELOCITY

    # Using the normalized curvature data to calculate cone distance
    for i in range(len(cone_distance_data)):
        cone_distance_data[i] = (1 - cone_distance_data[i]) * CONE_DISTANCE

    # Combining the track points and orientation into poses
    poses = [Pose(points[i], orientation[i]) for i in range(len(points))]
    velocity_data = velocity_data * MOTION_SAMPLING

    # Making the poses list more sparse depending on the sampling rate and velocity
    sparse_poses, sparse_pose_indices = select_data(poses, velocity_data)
    velocity_data = velocity_data / MOTION_SAMPLING

    # Creating motion data out of the sparse poses
    motion: list[Motion] = []

    for i in range(len(sparse_poses) - 1):
        # Calculating the angular velocity
        angle1 = sparse_poses[i + 1].get_orientation()
        angle2 = sparse_poses[i].get_orientation()
        if np.copysign(1, angle1) == np.copysign(1, angle2):
            difference = angle1 - angle2
        else:
            dominant = angle1 if np.pi - abs(angle1) > np.pi - abs(angle2) else angle2
            difference = abs(abs(angle1) - abs(angle2))
            np.copysign(difference, dominant)
        motion.append(
            Motion(
                velocity_data[sparse_pose_indices[i]],
                difference / MOTION_SAMPLING
            )
        )

    motion.append(Motion(motion[-1].get_speed(), motion[-1].get_angular_velocity()))

    # Making the poses list more sparse depending on the sampling rates
    x = [p.get_coord().get_x() for i, p in enumerate(sparse_poses) if i % int(PERCEPTION_SAMPLING / MOTION_SAMPLING) == 0]
    y = [p.get_coord().get_y() for i, p in enumerate(sparse_poses) if i % int(PERCEPTION_SAMPLING / MOTION_SAMPLING) == 0]

    # plt.plot(x, y, '.', color='black')

    if directions:
        for i in range(len(sparse_poses)):
            x2 = sparse_poses[i].get_coord().get_x() + 5 * np.cos(sparse_poses[i].get_orientation())
            y2 = sparse_poses[i].get_coord().get_y() + 5 * np.sin(sparse_poses[i].get_orientation())
            plt.plot(
                [sparse_poses[i].get_coord().get_x(), x2],
                [sparse_poses[i].get_coord().get_y(), y2],
                color='green',
            )

    # Calculating the first derivative of x and y
    gradient_x = np.gradient(x)
    gradient_y = np.gradient(y)

    # Calculating the cone positions with the help of the gradient
    left_side_points: list[Point] = []
    right_side_points: list[Point] = []

    for i in range(len(x)):
        gradient_norm = np.sqrt(gradient_x[i] * gradient_x[i] + gradient_y[i] * gradient_y[i])
        left_side_points.append(Point(
            x[i] + TRACK_HALF_WIDTH * (-gradient_y[i] / gradient_norm),
            y[i] + TRACK_HALF_WIDTH * (gradient_x[i] / gradient_norm)
        ))
        right_side_points.append(Point(
            x[i] - TRACK_HALF_WIDTH * (-gradient_y[i] / gradient_norm),
            y[i] - TRACK_HALF_WIDTH * (gradient_x[i] / gradient_norm)
        ))

    # Interpolating the cone positions to be more dense and then 
    # making it sparse again depending on the cone distance data
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
        color='blue',
        label='blue cone'
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
        color='orange',
        label='yellow cone'
    )

    all_cones = [*sparse_blue_cones, *sparse_yellow_cones]

    # Calculating the actual poses from the motion data
    actual_poses = np.zeros((len(motion), 3))
    ts = MOTION_SAMPLING

    for i in range(1, np.size(actual_poses, 0)):
        actual_poses[i, 0] = actual_poses[i - 1, 0] + motion[i].get_speed() * ts * np.cos(actual_poses[i - 1, 2] + motion[i].get_angular_velocity() * ts / 2)
        actual_poses[i, 1] = actual_poses[i - 1, 1] + motion[i].get_speed() * ts * np.sin(actual_poses[i - 1, 2] + motion[i].get_angular_velocity() * ts / 2)
        actual_poses[i, 2] = actual_poses[i - 1, 2] + motion[i].get_angular_velocity() * ts

    plt.plot(actual_poses[:, 0], actual_poses[:, 1], '.', color='purple', label='actual pose')

    # Writing the simulation data to a file
    write_data_to_file(
        f'output_{filename}.txt',
        cones=all_cones.copy(),
        poses=actual_poses.copy(),
        motion=motion.copy(),
        noisy=False,
        perception=perception
    )
    if noisy:
        write_data_to_file(
            f'output_{filename}_noisy.txt',
            cones=all_cones.copy(),
            poses=actual_poses.copy(),
            motion=motion.copy(),
            noisy=True,
            perception=False
        )

    with open(f'{filename}_poses_gt.txt', 'w') as oF:
        for p in actual_poses:
            print(p[0], p[1], p[2], file=oF)

    plt.xlabel('x[m]')
    plt.ylabel('y[m]')
    plt.title('Simulation')
    plt.legend()
    plt.axis('equal')
    plt.savefig('track.png', dpi=500)
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Perception simulator for driverless cars.', allow_abbrev=False)
    parser.add_argument('input_file', type=str, nargs=1, help='path to the file with sparse centerline data')
    parser.add_argument('-d', '--directions', action='store_true', default=False, required=False, help='show the directional vectors of the car')
    parser.add_argument('-n', '--noisy', action='store_true', default=False, required=False, help='use noise for the output data')
    parser.add_argument('-p', '--perception', action='store_true', default=False, required=False, help='show the vectors to the cones the car is currently detecting')
    args = parser.parse_args()
    main(path=args.input_file[0], directions=args.directions, noisy=args.noisy, perception=args.perception)
