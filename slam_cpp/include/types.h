#ifndef TYPES_H
#define TYPES_H

#include <iomanip>

#include <Eigen/Core>

#include "utils.h"

#define TIMESTAMP 0.05 // 50 ms

struct Landmark
{
    double data[2];
    double ground_truth[2];
    int color;
    int id;

    Landmark(int id = -1, double x = 0, double y = 0, int color = 0, double glob_x = 0, double glob_y = 0):
        data{x, y}, ground_truth{glob_x, glob_y}, color(color), id(id) {}

    friend std::ostream& operator<<(std::ostream& os, const Landmark& obj)
    {
        os << std::setprecision(16);
        os << obj.id << ' ' << obj.data[0] << ' ' << obj.data[1] << ' '
            << obj.color << ' ' << obj.ground_truth[0] << ' ' << obj.ground_truth[1];

        return os;
    }
};

struct Odometry
{
    double data[2];

    Odometry(double velocity = 0, double angular_velocity = 0): data{velocity, angular_velocity} {}

    friend std::ostream& operator<<(std::ostream& os, const Odometry& obj)
    {
        os << std::setprecision(16);
        os << obj.data[0] << ' ' << obj.data[1];

        return os;
    }
};

struct Perception
{
    double data[2];
    double ground_truth[2];
    int color;
    int id;

    Perception(int id = -1, double distance = 0, double bearing = 0, int color = 0, double glob_x = 0, double glob_y = 0):
        data{distance, bearing}, ground_truth{glob_x, glob_y}, color(color), id(id) {}

    friend std::ostream& operator<<(std::ostream& os, const Perception& obj)
    {
        os << std::setprecision(16);
        os << obj.id << ' ' << obj.data[0] << ' ' << obj.data[1] << ' '
            << obj.color << ' ' << obj.ground_truth[0] << ' ' << obj.ground_truth[1];

        return os;
    }
};

struct Pose
{
    double data[3];

    Pose(double x = 0, double y = 0, double psi = 0): data{x, y, psi} {}

    Pose& operator+=(Odometry& obj)
    {
        data[0] += obj.data[0] * TIMESTAMP * ceres::cos(data[2] + obj.data[1] * TIMESTAMP * 0.5);
        data[1] += obj.data[0] * TIMESTAMP * ceres::sin(data[2] + obj.data[1] * TIMESTAMP * 0.5);
        data[2] = data[2] + obj.data[1] * TIMESTAMP;

        return *this;
    }
    friend std::ostream& operator<<(std::ostream& os, const Pose& obj)
    {
        os << std::setprecision(16);
        os << obj.data[0] << ' ' << obj.data[1] << ' ' << obj.data[2];

        return os;
    }
};

#endif // TYPES_H
