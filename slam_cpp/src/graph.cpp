#include "../include/graph.h"

int Graph::m_last_id = 0;

Graph::Graph(const Pose& pose)
{
    m_poses[m_last_id++] = Pose(pose);
}

void Graph::createPose(const Pose& measurement)
{
    std::map<int, Pose>::iterator prev = std::prev(m_poses.end());

    m_poses[m_last_id++] = Pose(prev->second + measurement);
    m_pose_measurements.push_back(measurement);

    std::map<int, Pose>::iterator curr = std::prev(m_poses.end());

    Pose& meas = m_pose_measurements.back();

    std::cout << prev->second << std::endl;
    std::cout << meas << std::endl;
    std::cout << curr->second << std::endl;

    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<PoseErrorFunction, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1>(new PoseErrorFunction);

    m_problem.AddResidualBlock(cost_function, nullptr, 
                                &prev->second.m_x, &prev->second.m_y, &prev->second.m_psi,
                                &curr->second.m_x, &curr->second.m_y, &curr->second.m_psi,
                                &meas.m_x, &meas.m_y, &meas.m_psi);
    m_problem.SetParameterBlockConstant(&meas.m_x);
    m_problem.SetParameterBlockConstant(&meas.m_y);
    m_problem.SetParameterBlockConstant(&meas.m_psi);
}

bool Graph::optimize(bool report)
{
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;
    // options.minimizer_progress_to_stdout = true;

    std::map<int, Pose>::iterator start = m_poses.begin();

    m_problem.SetParameterBlockConstant(&start->second.m_x);
    m_problem.SetParameterBlockConstant(&start->second.m_y);
    m_problem.SetParameterBlockConstant(&start->second.m_psi);

    Solve(options, &m_problem, &summary);

    if (report)
        std::cout << summary.FullReport() << std::endl;

    return summary.IsSolutionUsable();
}

std::ostream& operator<<(std::ostream& os, const Graph& graph)
{
    std::map<int, Pose>::const_iterator it;

    for (it = graph.m_poses.begin(); it != graph.m_poses.end(); ++it)
    {
        os << it->first << ' ' << it->second << std::endl;
    }

    return os;
}
