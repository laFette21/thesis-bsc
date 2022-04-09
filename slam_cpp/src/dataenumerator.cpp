#include "../include/dataenumerator.h"

std::set<int> Data::getLandmarkIdsFromPerceptions(const std::vector<Perception>& perceptions)
{
    std::set<int> result;

    for (auto& perception : perceptions)
        result.insert(perception.id);

    return result;
}

std::ostream& operator<<(std::ostream& os, const Data& obj)
{
    os << std::setprecision(16);
    os << "motion: " << obj.motion << std::endl;

    for (auto& perception : obj.perceptions)
        os << "perception: " << perception << std::endl; 

    return os;
}

DataEnumerator::DataEnumerator(const std::string& str)
{
    m_file.open(str);
    if(m_file.fail()) throw std::runtime_error("OPEN ERROR");
    m_end = false;
}

void DataEnumerator::first()
{
    m_end = !read_next_not_empty_line();
    next();
}

void DataEnumerator::next()
{
    char type;
    m_ss >> type;

    if (!m_ss.fail())
    {
        std::vector<Perception> perceptions;
        Motion motion;
        double vel = 0, ang_vel = 0, count = 0;
        double trash;

        while (!m_end && type == 'o')
        {
            m_ss >> trash >> vel >> ang_vel;

            motion.data[0] += vel;
            motion.data[1] += ang_vel;
            count += 1;

            m_end = !read_next_not_empty_line();
            m_ss >> type;
        }

        motion.data[0] /= count;
        motion.data[1] /= count;

        // TODO: zajgenerálás
        std::random_device rd; 
        std::mt19937 gen(rd()); 
        std::normal_distribution<double> normal(0, 2.0);

        motion.data[0] += motion.data[0] * normal(gen);
        motion.data[1] += motion.data[1] * normal(gen);

        while (!m_end && type == 'p')
        {
            Perception perception;

            m_ss >> trash >> perception.data[0] >> perception.data[1] >> perception.color
                >> perception.id >> perception.ground_truth[0] >> perception.ground_truth[1];
            perceptions.push_back(perception);

            m_end = !read_next_not_empty_line();
            m_ss >> type;
        }

        std::stringstream temp;
        temp << 'o';
        temp << m_ss.rdbuf();
        m_ss = std::move(temp);

        m_data.motion = motion;
        m_data.perceptions = perceptions;
    }
}

bool DataEnumerator::read_next_not_empty_line()
{
    std::string line;
    m_ss.clear();
    while (getline(m_file, line) && line.size() == 0);
    if (!(m_file.fail())) m_ss.str(line);
    return !m_file.fail();
}
