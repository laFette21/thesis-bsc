#include "../include/graph.h"

int Graph::m_last_id = 0;

Graph::Graph(const Pose& pose)
{
    m_pose_cost_function = new ceres::AutoDiffCostFunction<PoseErrorFunction, 3, 3, 3, 3>(new PoseErrorFunction);
    m_poses[m_last_id++] = std::shared_ptr<Pose>(new Pose(pose));
}

void Graph::createPose(const std::shared_ptr<Pose>& measurement)
{
    std::map<int, std::shared_ptr<Pose>>::iterator prev = std::prev(m_poses.end());

    m_poses[m_last_id++] = std::shared_ptr<Pose>(new Pose(*prev->second * *measurement));
    m_pose_measurements.push_back(measurement);

    std::map<int, std::shared_ptr<Pose>>::iterator curr = std::prev(m_poses.end());

    std::shared_ptr<Pose> meas = m_pose_measurements.back();

    std::cout << *prev->second << std::endl;
    std::cout << *meas << std::endl;
    std::cout << *curr->second << std::endl;

    m_problem.AddResidualBlock(m_pose_cost_function, nullptr, prev->second->m_data, curr->second->m_data, meas->m_data);
    m_problem.SetParameterBlockConstant(meas->m_data);
}

bool Graph::optimize(bool report)
{
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // options.minimizer_progress_to_stdout = true;

    std::map<int, std::shared_ptr<Pose>>::iterator start = m_poses.begin();

    m_problem.SetParameterBlockConstant(start->second->m_data);

    Solve(options, &m_problem, &summary);

    if (report)
        std::cout << summary.FullReport() << std::endl;

    return summary.IsSolutionUsable();
}

std::ostream& operator<<(std::ostream& os, const Graph& graph)
{
    std::map<int, std::shared_ptr<Pose>>::const_iterator it;

    for (it = graph.m_poses.begin(); it != graph.m_poses.end(); ++it)
    {
        os << it->first << ' ' << *it->second << std::endl;
    }

    return os;
}
