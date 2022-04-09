#ifndef GRAPH_H
#define GRAPH_H

#include <map>

#include <ceres/ceres.h>

#include "types.h"
#include "utils.h"

enum DebugOption { POSE, LANDMARK, BOTH };

class Graph
{
    ceres::CostFunction *m_pose_cost_function, *m_landmark_cost_function;
    ceres::Solver::Options m_options;
    std::map<int, std::shared_ptr<Pose>> m_poses;
    std::map<int, std::vector<std::shared_ptr<Landmark>>> m_landmarks;
    Pose m_prev_global_pose;
    std::map<int, std::shared_ptr<Landmark>> m_unique_landmarks;
    std::map<int, std::shared_ptr<Motion>> m_pose_measurements;
    std::map<int, std::shared_ptr<std::vector<std::shared_ptr<Perception>>>> m_landmark_measurements;
    std::vector<ceres::ResidualBlockId> m_pose_residual_ids;
    std::vector<ceres::ResidualBlockId> m_lm_residual_ids;
    static int m_last_id;

public:
    Graph();

    void createLandmark(const std::shared_ptr<std::vector<std::shared_ptr<Perception>>>&);
    void createPose(const std::shared_ptr<Motion>&);
    bool optimize(int = -1, bool = false);
    std::vector<double> debug(ceres::Problem&, const DebugOption&);
    std::map<int, std::shared_ptr<Landmark>> getUniqueLandmarks() const { return m_unique_landmarks; }

    friend std::ostream& operator<<(std::ostream&, const Graph&);
};

#endif // GRAPH_H
