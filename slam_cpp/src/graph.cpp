#include "../include/graph.h"

int Graph::m_last_id = 0;

Graph::Graph()
{
    m_pose_cost_function = new ceres::AutoDiffCostFunction<PoseErrorFunction, 3, 3, 3, 2>(new PoseErrorFunction);
    m_landmark_cost_function = new ceres::AutoDiffCostFunction<LandmarkErrorFunction, 2, 3, 2, 2>(new LandmarkErrorFunction);
    m_poses[m_last_id++] = std::shared_ptr<Pose>(new Pose());
}

void Graph::createLandmark(const std::shared_ptr<std::vector<std::shared_ptr<Perception>>>& measurements)
{
    std::map<int, std::shared_ptr<Pose>>::iterator curr = std::prev(m_poses.end());

    for (auto& measurement : *measurements)
    {
        std::shared_ptr<Landmark> lm(new Landmark(measurement->id, 0, 0, measurement->color));

        if (!m_first_global_landmarks.count(measurement->id))
        {
            lm->data[0] = m_prev_global_pose.data[0] + measurement->data[0] * ceres::cos(measurement->data[1] + m_prev_global_pose.data[2]);
            lm->data[1] = m_prev_global_pose.data[1] + measurement->data[0] * ceres::sin(measurement->data[1] + m_prev_global_pose.data[2]);

            m_first_global_landmarks[measurement->id] = lm;
        }
        else
        {
            lm->data[0] = m_first_global_landmarks[measurement->id]->data[0];
            lm->data[1] = m_first_global_landmarks[measurement->id]->data[1];
        }

        m_landmarks[m_last_id].push_back(lm);
        m_landmark_measurements.push_back(measurements);

        m_problem.AddResidualBlock(m_landmark_cost_function, nullptr, curr->second->data, lm->data, measurement->data);
        m_problem.SetParameterBlockConstant(measurement->data);
    }
}

void Graph::createPose(const std::shared_ptr<Odometry>& measurement)
{
    std::map<int, std::shared_ptr<Pose>>::iterator prev = std::prev(m_poses.end());

    m_poses[m_last_id++] = std::shared_ptr<Pose>(new Pose(m_prev_global_pose));
    m_pose_measurements.push_back(measurement);

    std::map<int, std::shared_ptr<Pose>>::iterator curr = std::prev(m_poses.end());

    m_problem.AddResidualBlock(m_pose_cost_function, nullptr, prev->second->data, curr->second->data, measurement->data);
    m_problem.SetParameterBlockConstant(measurement->data);

    m_prev_global_pose += *measurement;
}

bool Graph::optimize(bool report)
{
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    options.max_num_iterations = 200;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // options.minimizer_progress_to_stdout = true;

    std::map<int, std::shared_ptr<Pose>>::iterator start = m_poses.begin();

    m_problem.SetParameterBlockConstant(start->second->data);

    Solve(options, &m_problem, &summary);

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
