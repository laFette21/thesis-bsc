#include "Graph.h"

/**
 * @brief Construct a new Graph object
 * 
 */
Graph::Graph()
{
    _last_id = 0;
    _poses[_last_id++] = PosePtr(new Pose);
    _options.max_num_iterations = 500;
    _options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
}

/**
 * @brief Create new Landmark objects.
 * 
 * Create new Landmark objects from measurements and add them to the graph
 * if there was no landmark with the same id. If there was a landmark with
 * the same id, no new landmark is created and the old landmark is added to
 * the graph again.
 * 
 * @param measurements
 */
void Graph::createLandmark(const std::shared_ptr<std::vector<PerceptionPtr>>& measurements)
{
    for (const auto& measurement : *measurements)
    {
        LandmarkPtr lm;

        if (!_unique_landmarks.count(measurement->id))
        {
            lm = LandmarkPtr(
                new Landmark(measurement->id, 0, 0, measurement->color, measurement->ground_truth[0], measurement->ground_truth[1])
            );

            lm->data[0] = _prev_global_pose.data[0] + measurement->data[0] * ceres::cos(measurement->data[1] + _prev_global_pose.data[2]);
            lm->data[1] = _prev_global_pose.data[1] + measurement->data[0] * ceres::sin(measurement->data[1] + _prev_global_pose.data[2]);

            _unique_landmarks[measurement->id] = lm;
        }
        else
        {
            lm = _unique_landmarks[measurement->id];
        }

        _landmarks[_last_id].push_back(lm);
    }

    _landmark_measurements[_last_id] = measurements;
}

/**
 * @brief Create new Pose objects.
 * 
 * Create new Pose objects from measurements and add them to the graph.
 * Measurements are modified with some constants from the PoseErrorFunction equation.
 * 
 * @param measurement
 */
void Graph::createPose(const MotionPtr& measurement)
{
    _prev_global_pose += *measurement;

    measurement->data[0] *= timestamp;
    measurement->data[1] *= timestamp * 0.5;
    _pose_measurements[_last_id] = measurement;

    _poses[_last_id++] = PosePtr(new Pose(_prev_global_pose));
}

/**
 * @brief Optimize the graph.
 * 
 * The method builds up the optimization problem with the help of Ceres
 * AutoDiffCostFunctions and solves it.
 * 
 * @param quantity
 * @param report
 * @return true if solution is usable else false.
 */
bool Graph::optimize(int quantity, bool report)
{
    ceres::Problem problem;
    ceres::Solver::Summary summary;

    _pose_cost_function = new ceres::AutoDiffCostFunction<PoseErrorFunction, 3, 3, 3, 2>(new PoseErrorFunction);
    _landmark_cost_function = new ceres::AutoDiffCostFunction<LandmarkErrorFunction, 2, 3, 2, 2>(new LandmarkErrorFunction);

    if (quantity < 0)
        quantity = _poses.size();

    auto start = std::prev(_poses.end(), quantity);
    auto prev = start;
    auto curr = std::next(prev);

    while (curr != _poses.end())
    {
        problem.AddResidualBlock(
            _pose_cost_function,
            nullptr,
            prev->second->data,
            curr->second->data,
            _pose_measurements[curr->first]->data
        );
        problem.SetParameterBlockConstant(_pose_measurements[curr->first]->data);

        auto measurements = _landmark_measurements[curr->first];

        // sorfolytonos adattárolás (vector) -> cache friendly
        for (const auto& lm : _landmarks[curr->first])
        {
            auto is_id = [lm](PerceptionPtr obj){ return obj->id == lm->id; };
            auto meas = std::find_if(measurements->begin(), measurements->end(), is_id);

            problem.AddResidualBlock(
                _landmark_cost_function, nullptr, prev->second->data, lm->data, meas->get()->data
            );
            problem.SetParameterBlockConstant(meas->get()->data);
        }

        prev++;
        curr++;
    }

    // Set anchor for first pose
    problem.SetParameterBlockConstant(start->second->data);

    Solve(_options, &problem, &summary);

    if (report)
        std::cerr << summary.FullReport() << std::endl;

    return summary.IsSolutionUsable();
}

/**
 * @brief Print the Pose objects from the Graph to the stream.
 * 
 * @return std::ostream&
 */
// GCOVR_EXCL_START
std::ostream& operator<<(std::ostream& os, const Graph& graph)
{
    for (const auto& pose : graph._poses)
    {
        os << pose.first << ' ' << *pose.second << std::endl;
    }

    return os;
}
// GCOVR_EXCL_STOP
