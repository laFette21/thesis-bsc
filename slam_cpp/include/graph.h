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
    std::map<int, Pose> m_poses;
    std::vector<Pose> m_pose_measurements;

public:
    Graph(const Pose&);

    void createPose(const Pose&);
    // void addLandmark(const Pose&, const Landmark&);
    bool optimize(bool = false);

    friend std::ostream& operator<<(std::ostream&, const Graph&);
};

#endif // GRAPH_H
