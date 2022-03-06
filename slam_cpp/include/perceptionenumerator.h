#ifndef PERCEPTIONENUMERATOR_H
#define PERCEPTIONENUMERATOR_H

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

#include "types.h"

struct Perception
{
    std::vector<Landmark> m_landmarks;
    Pose m_pose;

    Perception() {}
    Perception(const Pose& pose, const std::vector<Landmark>& landmarks): m_pose(pose), m_landmarks(landmarks) {}

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
