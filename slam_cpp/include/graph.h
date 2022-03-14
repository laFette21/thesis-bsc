#ifndef GRAPH_H
#define GRAPH_H

#include <map>

#include <ceres/ceres.h>

#include "types.h"
#include "utils.h"

class Graph
{
    ceres::Problem m_problem;
    ceres::CostFunction *m_pose_cost_function, *m_landmark_cost_function;
    std::map<int, std::shared_ptr<Pose>> m_poses;
    std::map<int, std::vector<std::shared_ptr<Landmark>>> m_landmarks;
    Pose m_prev_global_pose;
    std::vector<std::shared_ptr<Odometry>> m_pose_measurements;
    std::vector<std::shared_ptr<std::vector<std::shared_ptr<Perception>>>> m_landmark_measurements;
    static int m_last_id;

public:
    Graph();

    void createLandmark(const std::shared_ptr<std::vector<std::shared_ptr<Perception>>>&);
    void createPose(const std::shared_ptr<Odometry>&);
    bool optimize(bool = false);

    friend std::ostream& operator<<(std::ostream&, const Graph&);
};

#endif // GRAPH_H
