#ifndef PERCEPTIONENUMERATOR_H
#define PERCEPTIONENUMERATOR_H

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

struct Pose
{
    double m_x;
    double m_y;
    double m_psi;
    int m_id;

    Pose(int id = -1, double x = 0, double y = 0, double psi = 0): m_id(id), m_x(x), m_y(y), m_psi(psi) {}
};

struct Landmark
{
    double m_distance;
    double m_bearing;
    int m_color;
    int m_id;

    Landmark(int id = -1, double distance = 0, double bearing = 0, int color = 0):
        m_id(id), m_distance(distance), m_bearing(bearing), m_color(color) {}
};

struct Perception
{
    std::vector<Landmark> m_landmarks;
    Pose m_pose;

    Perception() {}
    Perception(Pose& pose, std::vector<Landmark>& landmarks): m_pose(pose), m_landmarks(landmarks) {}
    friend std::ostream& operator<<(std::ostream&, const Perception&);
};

class PerceptionEnumerator
{
    std::ifstream m_file;
    std::stringstream m_ss;
    Perception m_data;
    bool m_end;

public:
    PerceptionEnumerator(const std::string&);
    void first();
    void next();
    bool end() const { return m_end; }
    Perception current() const { return m_data; }
    bool read_next_not_empty_line();
};

#endif // PERCEPTIONENUMERATOR_H
