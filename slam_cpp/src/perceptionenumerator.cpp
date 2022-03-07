#include "../include/perceptionenumerator.h"


std::ostream& operator<<(std::ostream& os, const Perception& per)
{
    os << std::setprecision(16);
    os << "pose: " << per.m_pose << std::endl;

    for (size_t i = 0; i < per.m_landmarks.size(); ++i)
        os << "lm: " << per.m_landmarks[i] << std::endl; 

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
    char l_type;
    m_ss >> l_type;

    if (!m_ss.fail())
    {
        std::vector<Landmark> l_landmarks;
        Pose l_pose;

        m_ss >> l_pose.m_data[0] >> l_pose.m_data[1] >> l_pose.m_data[2];

        m_end = !read_next_not_empty_line();
        m_ss >> l_type;

        double l_distance, l_bearing;
        int l_color, l_id;

        while (!m_end && l_type == 'm')
        {
            Landmark l_landmark;

            m_ss >> l_distance >> l_bearing >> l_color >> l_id;
            l_landmark.m_bearing = l_bearing;
            l_landmark.m_color = l_color;
            l_landmark.m_distance = l_distance;
            l_landmark.m_id = l_id;
            l_landmarks.push_back(l_landmark);

            m_end = !read_next_not_empty_line();
            m_ss >> l_type;
        }

        std::stringstream l_temp;
        l_temp << 'p';
        l_temp << m_ss.rdbuf();
        m_ss = std::move(l_temp);

        m_data.m_pose = l_pose;
        m_data.m_landmarks = l_landmarks;
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
