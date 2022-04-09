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

void Graph::createPose(const std::shared_ptr<Motion>& measurement)
{
    m_prev_global_pose += *measurement;

    measurement->data[0] *= TIMESTAMP;
    measurement->data[1] *= TIMESTAMP * 0.5;
    m_pose_measurements[m_last_id] = measurement;

    m_poses[m_last_id++] = std::shared_ptr<Pose>(new Pose(m_prev_global_pose));
}

std::vector<double> Graph::debug(ceres::Problem& problem, const DebugOption& debug_option)
{
    double total_cost = 0.0;
    std::vector<double> evaluated_residuals;
    ceres::Problem::EvaluateOptions options;

    if (debug_option == DebugOption::POSE)
        options.residual_blocks = m_pose_residual_ids;
    else if (debug_option == DebugOption::LANDMARK)
        options.residual_blocks = m_lm_residual_ids;
    else
    {
        options.residual_blocks = m_pose_residual_ids;
        options.residual_blocks.insert(options.residual_blocks.end(), m_lm_residual_ids.begin(), m_lm_residual_ids.end());
    }

    problem.Evaluate(options, &total_cost, &evaluated_residuals, nullptr, nullptr);

    // TODO: Pontosan mi a residual itt? Residual tömb, vagy valami normált érték?

    return evaluated_residuals;
}

bool Graph::optimize(int quantity, bool report)
{
    ceres::Problem problem;
    ceres::Solver::Summary summary;

    m_pose_cost_function = new ceres::AutoDiffCostFunction<PoseErrorFunction, 3, 3, 3, 2>(new PoseErrorFunction);
    m_landmark_cost_function = new ceres::AutoDiffCostFunction<LandmarkErrorFunction, 2, 3, 2, 2>(new LandmarkErrorFunction);

    if (quantity < 0)
        quantity = m_poses.size();

    auto start = std::prev(m_poses.end(), quantity);
    auto prev = start;
    auto curr = std::next(prev);

    while (curr != m_poses.end())
    {
        ceres::ResidualBlockId pose_residual_id = problem.AddResidualBlock(
            m_pose_cost_function,
            nullptr,
            prev->second->data,
            curr->second->data,
            m_pose_measurements[curr->first]->data
        );
        problem.SetParameterBlockConstant(m_pose_measurements[curr->first]->data);
        m_pose_residual_ids.push_back(pose_residual_id);

        auto measurements = *m_landmark_measurements[curr->first];

        // TODO: optimize performance?
        // sorfolytonos adattárolás (vector) -> cache friendly
        for (auto& lm : m_landmarks[curr->first])
        {
            auto is_id = [lm](std::shared_ptr<Perception> obj){ return obj->id == lm->id; };
            auto meas = std::find_if(measurements.begin(), measurements.end(), is_id);

            ceres::ResidualBlockId lm_residual_id = problem.AddResidualBlock(
                m_landmark_cost_function, nullptr, prev->second->data, lm->data, meas->get()->data
            );
            problem.SetParameterBlockConstant(meas->get()->data);
            m_lm_residual_ids.push_back(lm_residual_id);
        }

        prev++;
        curr++;
    }

    // Set anchor for first pose
    problem.SetParameterBlockConstant(start->second->data);
/*
    auto residuals = debug(problem, DebugOption::BOTH);
    double total_cost = 0;

    for (auto& residual : residuals)
    {
        // std::cerr << residual << std::endl;
        total_cost += residual * residual;
    }

    std::cout << "Total cost before optimization: " << total_cost / 2 << std::endl;

    // return false;
*/
    Solve(m_options, &problem, &summary);
/*
    residuals = debug(problem, DebugOption::BOTH);
    total_cost = 0;

    for (auto& residual : residuals)
    {
        // std::cerr << residual << std::endl;
        total_cost += residual * residual;
    }

    std::cout << "Total cost after optimization: " << total_cost / 2 << std::endl;
*/
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
