#include "../include/perceptionenumerator.h"


std::ostream& operator<<(std::ostream& os, const Perception& obj)
{
    os << std::setprecision(16);
    os << "odometry: " << obj.odometry << std::endl;

    for (size_t i = 0; i < obj.landmarks.size(); ++i)
        os << "lm: " << obj.landmarks[i] << std::endl; 

    return os;
}

PerceptionEnumerator::PerceptionEnumerator(const std::string& str)
{
    m_file.open(str);
    if(m_file.fail()) throw std::runtime_error("OPEN ERROR");
    m_end = false;
}

void PerceptionEnumerator::first()
{
    m_end = !read_next_not_empty_line();
    next();
}

void PerceptionEnumerator::next()
{
    char type;
    m_ss >> type;

    if (!m_ss.fail())
    {
        std::vector<Landmark> landmarks;
        Odometry odometry;
        double vel = 0, ang_vel = 0, count = 0;
        double trash;

        while (!m_end && type == 'o')
        {
            m_ss >> trash >> vel >> ang_vel;

            odometry.data[0] += vel;
            odometry.data[1] += ang_vel;
            count += 1;

            m_end = !read_next_not_empty_line();
            m_ss >> type;
        }

        odometry.data[0] /= count;
        odometry.data[1] /= count;

        while (!m_end && type == 'p')
        {
            Landmark landmark;

            m_ss >> trash >> landmark.distance >> landmark.bearing >> landmark.color >> landmark.id;
            landmarks.push_back(landmark);

            m_end = !read_next_not_empty_line();
            m_ss >> type;
        }

        std::stringstream temp;
        temp << 'o';
        temp << m_ss.rdbuf();
        m_ss = std::move(temp);

        m_data.odometry = odometry;
        m_data.landmarks = landmarks;
    }
}

bool PerceptionEnumerator::read_next_not_empty_line()
{
    std::string line;
    m_ss.clear();
    while (getline(m_file, line) && line.size() == 0);
    if (!(m_file.fail())) m_ss.str(line);
    return !m_file.fail();
}
