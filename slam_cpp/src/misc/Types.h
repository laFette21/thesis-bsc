#ifndef TYPES_H
#define TYPES_H

#include <iomanip>

static constexpr double timestamp = 0.05; // 50 ms

struct Landmark
{
    double data[2];
    double ground_truth[2];
    int color;
    int id;

    Landmark(int id = -1, double x = 0, double y = 0, int color = 0, double glob_x = 0, double glob_y = 0):
        data{x, y}, ground_truth{glob_x, glob_y}, color(color), id(id) {}

    bool operator==(const Landmark&) const = default;

    friend std::ostream& operator<<(std::ostream& os, const Landmark& obj)
    {
        os << std::setprecision(16);
        os << obj.id << ' ' << obj.data[0] << ' ' << obj.data[1] << ' '
            << obj.color << ' ' << obj.ground_truth[0] << ' ' << obj.ground_truth[1];

        return os;
    }
};

struct Motion
{
    double data[2];

    Motion(double velocity = 0, double angular_velocity = 0): data{velocity, angular_velocity} {}

    bool operator==(const Motion&) const = default;

    friend std::ostream& operator<<(std::ostream& os, const Motion& obj)
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

    bool operator==(const Perception&) const = default;

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

    Pose& operator+=(const Motion& obj)
    {
        data[0] += obj.data[0] * timestamp * cos(data[2] + obj.data[1] * timestamp * 0.5);
        data[1] += obj.data[0] * timestamp * sin(data[2] + obj.data[1] * timestamp * 0.5);
        data[2] = data[2] + obj.data[1] * timestamp;

        return *this;
    }
    bool operator==(const Pose&) const = default;

    friend std::ostream& operator<<(std::ostream& os, const Pose& obj)
    {
        os << std::setprecision(16);
        os << obj.data[0] << ' ' << obj.data[1] << ' ' << obj.data[2];

        return os;
    }
};

#endif // TYPES_H
