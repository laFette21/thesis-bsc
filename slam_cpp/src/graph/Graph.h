#ifndef GRAPH_H
#define GRAPH_H

#include <map>

#include <ceres/ceres.h>

#include "Types.h"
#include "Utils.h"

enum DebugOption { POSE, LANDMARK, BOTH };

class Graph
{
    using LandmarkPtr = std::shared_ptr<Landmark>;
    using MotionPtr = std::shared_ptr<Motion>;
    using PerceptionPtr = std::shared_ptr<Perception>;
    using PosePtr = std::shared_ptr<Pose>;
public:
    Graph();

    void createLandmark(const std::shared_ptr<std::vector<PerceptionPtr>>&);
    void createPose(const MotionPtr&);
    bool optimize(int = -1, bool = false);
    std::vector<double> debug(ceres::Problem&, const DebugOption&);
    std::map<int, LandmarkPtr> getUniqueLandmarks() const { return _unique_landmarks; }

    friend std::ostream& operator<<(std::ostream&, const Graph&);

private:
    ceres::CostFunction *_pose_cost_function, *_landmark_cost_function;
    ceres::Solver::Options _options;
    std::map<int, PosePtr> _poses;
    std::map<int, std::vector<LandmarkPtr>> _landmarks;
    Pose _prev_global_pose;
    std::map<int, LandmarkPtr> _unique_landmarks;
    std::map<int, MotionPtr> _pose_measurements;
    std::map<int, std::shared_ptr<std::vector<PerceptionPtr>>> _landmark_measurements;
    std::vector<ceres::ResidualBlockId> _pose_residual_ids;
    std::vector<ceres::ResidualBlockId> _lm_residual_ids;
    static int _last_id;
};

#endif // GRAPH_H
