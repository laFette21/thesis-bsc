#ifndef TYPES_H
#define TYPES_H

#include <iomanip>

#include <Eigen/Core>

struct Pose
{
    double m_data[3];

    Pose(double x = 0, double y = 0, double psi = 0): m_data{x, y, psi} {}

    Pose operator+(const Pose& obj) { return Pose(m_data[0] + obj.m_data[0], m_data[1] + obj.m_data[1], m_data[2] + obj.m_data[2]); }
    friend std::ostream& operator<<(std::ostream& os, const Pose& pose)
    {
        os << std::setprecision(16);
        os << pose.m_data[0] << ' ' << pose.m_data[1] << ' ' << pose.m_data[2];

        return os;
    }
};

struct Landmark
{
    double m_distance;
    double m_bearing;
    int m_color;
    int m_id;

    Landmark(int id = -1, double distance = 0, double bearing = 0, int color = 0):
        m_id(id), m_distance(distance), m_bearing(bearing), m_color(color) {}

    friend std::ostream& operator<<(std::ostream& os, const Landmark& lm)
    {
        os << std::setprecision(16);
        os << lm.m_id << ' ' << lm.m_distance << ' ' << lm.m_bearing << ' ' << lm.m_color;

        return os;
    }
};

#endif // TYPES_H
