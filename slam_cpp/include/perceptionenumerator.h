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
    Odometry odometry;
    std::vector<Landmark> landmarks;

    Perception() {}
    Perception(const Odometry& odometry, const std::vector<Landmark>& landmarks):
        odometry(odometry), landmarks(landmarks) {}

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
