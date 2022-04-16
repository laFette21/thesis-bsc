#include "DataEnumerator.h"

/**
 * @brief Get landmark ids from perceptions.
 * 
 * @param perceptions
 * @return std::set<int> containing landmark ids.
 */
std::set<int> Data::getLandmarkIdsFromPerceptions(const std::vector<Perception>& perceptions)
{
    std::set<int> result;

    for (auto& perception : perceptions)
        result.insert(perception.id);

    return result;
}

/**
 * @brief Print the Data object to the stream.
 * 
 * @param os
 * @param obj
 * @return std::ostream&
 */
std::ostream& operator<<(std::ostream& os, const Data& obj)
{
    os << std::setprecision(16);
    os << "motion: " << obj.motion << std::endl;

    for (auto& perception : obj.perceptions)
        os << "perception: " << perception << std::endl; 

    return os;
}

/**
 * @brief Construct a new DataEnumerator object.
 * 
 * @param filename
 */
DataEnumerator::DataEnumerator(const std::string& filename)
{
    _file.open(filename);
    if(_file.fail()) throw std::runtime_error("OPEN ERROR");
    _end = false;
}

/**
 * @brief Read the first line from the file.
 * 
 */
void DataEnumerator::first()
{
    _end = !read_next_not_empty_line();
    next();
}

/**
 * @brief Process the read data.
 * 
 */
void DataEnumerator::next()
{
    char type;
    _ss >> type;

    if (!_ss.fail())
    {
        std::vector<Perception> perceptions;
        Motion motion;
        double vel = 0, ang_vel = 0, count = 0;
        double trash;

        while (!_end && type == 'o')
        {
            _ss >> trash >> vel >> ang_vel;

            motion.data[0] += vel;
            motion.data[1] += ang_vel;
            count += 1;

            _end = !read_next_not_empty_line();
            _ss >> type;
        }

        motion.data[0] /= count;
        motion.data[1] /= count;

        std::random_device rd; 
        std::mt19937 gen(rd()); 
        std::normal_distribution<double> normal(0, 2.0);

        motion.data[0] += motion.data[0] * normal(gen);
        motion.data[1] += motion.data[1] * normal(gen);

        while (!_end && type == 'p')
        {
            Perception perception;

            _ss >> trash >> perception.data[0] >> perception.data[1] >> perception.color
                >> perception.id >> perception.ground_truth[0] >> perception.ground_truth[1];
            perceptions.push_back(perception);

            _end = !read_next_not_empty_line();
            _ss >> type;
        }

        std::stringstream temp;
        temp << 'o';
        temp << _ss.rdbuf();
        _ss = std::move(temp);

        _data.motion = motion;
        _data.perceptions = perceptions;
    }
}

/**
 * @brief Read the next not empty line from the file.
 * 
 * @return true if process was successful else false.
 */
bool DataEnumerator::read_next_not_empty_line()
{
    std::string line;
    _ss.clear();
    while (getline(_file, line) && line.size() == 0);
    if (!(_file.fail())) _ss.str(line);
    return !_file.fail();
}
