#ifndef DATAENUMERATOR_H
#define DATAENUMERATOR_H

#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <set>
#include <sstream>
#include <vector>

#include "Types.h"

struct Data
{
    Motion motion;
    std::vector<Perception> perceptions;

    Data() {}
    Data(const Motion& motion, const std::vector<Perception>& perceptions):
        motion(motion), perceptions(perceptions) {}

    static std::set<int> getLandmarkIdsFromPerceptions(const std::vector<Perception>&);

    bool operator==(const Data&) const = default;

    friend std::ostream& operator<<(std::ostream&, const Data&);
};

class DataEnumerator
{
public:
    DataEnumerator(const std::string&, double, bool);

    Data current() const { return _data; }
    void first();
    void next();
    bool end() const { return _end; }

private:
    bool readNextNotEmptyLine();

    std::ifstream _file;
    std::stringstream _ss;
    std::mt19937 _rng;
    Data _data;
    bool _end;
    double _noise;
};

#endif // DATAENUMERATOR_H
