#include "../include/graph.h"

int Graph::m_last_id = 0;

Graph::Graph()
{
    m_poses[m_last_id++] = std::shared_ptr<Pose>(new Pose);
    m_options.max_num_iterations = 500;
    m_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
}

void Graph::createLandmark(const std::shared_ptr<std::vector<std::shared_ptr<Perception>>>& measurements)
{
    std::map<int, std::shared_ptr<Pose>>::iterator curr = std::prev(m_poses.end());

    for (auto& measurement : *measurements)
    {
        std::shared_ptr<Landmark> lm;

        if (!m_unique_landmarks.count(measurement->id))
        {
            lm = std::shared_ptr<Landmark>(
                new Landmark(measurement->id, 0, 0, measurement->color, measurement->ground_truth[0], measurement->ground_truth[1])
            );

            lm->data[0] = m_prev_global_pose.data[0] + measurement->data[0] * ceres::cos(measurement->data[1] + m_prev_global_pose.data[2]);
            lm->data[1] = m_prev_global_pose.data[1] + measurement->data[0] * ceres::sin(measurement->data[1] + m_prev_global_pose.data[2]);

            m_unique_landmarks[measurement->id] = lm;
        }
        else
        {
            lm = m_unique_landmarks[measurement->id];
        }

        m_landmarks[m_last_id].push_back(lm);
    }

    m_landmark_measurements[m_last_id] = measurements;
}

void Graph::createPose(const std::shared_ptr<Odometry>& measurement)
{
    m_prev_global_pose += *measurement;

    measurement->data[0] *= TIMESTAMP;
    measurement->data[1] *= TIMESTAMP;
    m_pose_measurements[m_last_id] = measurement;

    m_poses[m_last_id++] = std::shared_ptr<Pose>(new Pose(m_prev_global_pose));
}

bool Graph::optimize(int quantity, bool report)
{
    ceres::Problem problem;
    ceres::Solver::Summary summary;

    m_pose_cost_function = new ceres::AutoDiffCostFunction<PoseErrorFunction, 3, 3, 3, 2>(new PoseErrorFunction);
    m_landmark_cost_function = new ceres::AutoDiffCostFunction<LandmarkErrorFunction, 2, 3, 2, 2>(new LandmarkErrorFunction);

    auto start = std::prev(m_poses.end(), quantity);
    auto prev = start;
    auto curr = std::next(prev);

    while (curr != m_poses.end())
    {
        problem.AddResidualBlock(m_pose_cost_function, nullptr, prev->second->data, curr->second->data, m_pose_measurements[curr->first]->data);
        problem.SetParameterBlockConstant(m_pose_measurements[curr->first]->data);

        auto measurements = *m_landmark_measurements[curr->first];

        // TODO: optimize performance?
        for (auto& lm : m_landmarks[curr->first])
        {
            auto is_id = [lm](std::shared_ptr<Perception> obj){ return obj->id == lm->id; };
            auto meas = std::find_if(measurements.begin(), measurements.end(), is_id);

            problem.AddResidualBlock(m_landmark_cost_function, nullptr, prev->second->data, lm->data, meas->get()->data);
            problem.SetParameterBlockConstant(meas->get()->data);
        }

        prev++;
        curr++;
    }

    // Set anchor for first pose
    problem.SetParameterBlockConstant(start->second->data);

    Solve(m_options, &problem, &summary);

    if (report)
        std::cerr << summary.FullReport() << std::endl;

    return summary.IsSolutionUsable();
}

std::ostream& operator<<(std::ostream& os, const Graph& graph)
{
    for (auto& pose : graph.m_poses)
    {
        os << pose.first << ' ' << *pose.second << std::endl;
    }

    return os;
}
