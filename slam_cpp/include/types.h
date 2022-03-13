#ifndef TYPES_H
#define TYPES_H

#include <iomanip>

#include <Eigen/Core>

#include "utils.h"

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

struct Pose
{
    double data[3];

    Pose(double x = 0, double y = 0, double psi = 0): data{x, y, psi} {}

    friend std::ostream& operator<<(std::ostream& os, const Pose& obj)
    {
        os << std::setprecision(16);
        os << obj.data[0] << ' ' << obj.data[1] << ' ' << obj.data[2];

        return os;
    }
};

struct Landmark
{
    double distance;
    double bearing;
    int color;
    int id;

    Landmark(int id = -1, double distance = 0, double bearing = 0, int color = 0):
        id(id), distance(distance), bearing(bearing), color(color) {}

    friend std::ostream& operator<<(std::ostream& os, const Landmark& obj)
    {
        os << std::setprecision(16);
        os << obj.id << ' ' << obj.distance << ' ' << obj.bearing << ' ' << obj.color;

        return os;
    }
};

#endif // TYPES_H
