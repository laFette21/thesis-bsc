#ifndef GRAPH_H
#define GRAPH_H

#include <map>

#include <ceres/ceres.h>

#include "types.h"
#include "utils.h"

class Graph
{
    ceres::CostFunction *m_pose_cost_function, *m_landmark_cost_function;
    ceres::Solver::Options m_options;
    std::map<int, std::shared_ptr<Pose>> m_poses;
    std::map<int, std::vector<std::shared_ptr<Landmark>>> m_landmarks;
    Pose m_prev_global_pose;
    std::map<int, std::shared_ptr<Landmark>> m_unique_landmarks;
    std::map<int, std::shared_ptr<Odometry>> m_pose_measurements;
    std::map<int, std::shared_ptr<std::vector<std::shared_ptr<Perception>>>> m_landmark_measurements;
    static int m_last_id;

public:
    Graph();

    void createLandmark(const std::shared_ptr<std::vector<std::shared_ptr<Perception>>>&);
    void createPose(const std::shared_ptr<Odometry>&);
    bool optimize(int = -1, bool = false);

    std::map<int, std::shared_ptr<Landmark>> getUniqueLandmarks() const { return m_unique_landmarks; }

    friend std::ostream& operator<<(std::ostream&, const Graph&);
};

#endif // GRAPH_H
