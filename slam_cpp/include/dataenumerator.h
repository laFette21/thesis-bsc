#ifndef DATAENUMERATOR_H
#define DATAENUMERATOR_H

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

#include "types.h"

struct Data
{
    Odometry odometry;
    std::vector<Perception> perceptions;

    Data() {}
    Data(const Odometry& odometry, const std::vector<Perception>& perceptions):
        odometry(odometry), perceptions(perceptions) {}

    friend std::ostream& operator<<(std::ostream&, const Data&);
};

class DataEnumerator
{
    std::ifstream m_file;
    std::stringstream m_ss;
    Data m_data;
    bool m_end;

public:
    DataEnumerator(const std::string&);
    void first();
    void next();
    bool end() const { return m_end; }
    Data current() const { return m_data; }
    bool read_next_not_empty_line();
};

#endif // DATAENUMERATOR_H
