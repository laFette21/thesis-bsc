#ifndef GRAPH_H
#define GRAPH_H

#include <map>

#include <ceres/ceres.h>

#include "types.h"
#include "utils.h"

class Graph
{
    static int m_last_id;
    ceres::Problem m_problem;
    ceres::CostFunction* m_pose_cost_function;
    std::map<int, std::shared_ptr<Pose>> m_poses;
    std::vector<std::shared_ptr<Pose>> m_pose_measurements;

public:
    Graph(const Pose&);

    void createPose(const std::shared_ptr<Pose>&);
    // void addLandmark(const Pose&, const Landmark&);
    bool optimize(bool = false);

    friend std::ostream& operator<<(std::ostream&, const Graph&);
};

#endif // GRAPH_H
