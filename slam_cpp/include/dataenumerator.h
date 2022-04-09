#ifndef DATAENUMERATOR_H
#define DATAENUMERATOR_H

#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <set>
#include <sstream>
#include <vector>

#include "types.h"

struct Data
{
    Motion motion;
    std::vector<Perception> perceptions;

    Data() {}
    Data(const Motion& motion, const std::vector<Perception>& perceptions):
        motion(motion), perceptions(perceptions) {}
    static std::set<int> getLandmarkIdsFromPerceptions(const std::vector<Perception>&);

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
