#ifndef TYPES_H
#define TYPES_H

#include <iomanip>

#include <Eigen/Core>

struct Pose
{
    double m_x;
    double m_y;
    double m_psi;

    Pose(double x = 0, double y = 0, double psi = 0): m_x(x), m_y(y), m_psi(psi) {}
    // Eigen::Vector3d getAsVector3d() const { return Eigen::Vector3d(m_x, m_y, m_psi); }

    Pose operator+(const Pose& obj) { return Pose(m_x + obj.m_x, m_y + obj.m_y, m_psi + obj.m_psi); }
    friend std::ostream& operator<<(std::ostream& os, const Pose& pose)
    {
        os << std::setprecision(16);
        os << pose.m_x << ' ' << pose.m_y << ' ' << pose.m_psi;

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
    // Eigen::Vector2d getAsVector2d() const { return Eigen::Vector2d(m_distance, m_bearing); }

    friend std::ostream& operator<<(std::ostream& os, const Landmark& lm)
    {
        os << std::setprecision(16);
        os << lm.m_id << ' ' << lm.m_distance << ' ' << lm.m_bearing << ' ' << lm.m_color;

        return os;
    }
};

#endif // TYPES_H
