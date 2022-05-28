#ifndef GRAPH_H
#define GRAPH_H

#include <map>

#include <ceres/ceres.h>

#include "Types.h"
#include "Utils.h"

/**
 * @brief Class that represents a Graph object which is used to store and optimize the data.
 * 
 */
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
    std::map<int, LandmarkPtr> getUniqueLandmarks() const { return _unique_landmarks; }
    std::map<int, PosePtr> getPoses() const { return _poses; }
    int getLastId() const { return _last_id; }

    friend std::ostream& operator<<(std::ostream&, const Graph&);

private:
    ceres::CostFunction *_pose_cost_function, *_landmark_cost_function;
    ceres::Solver::Options _options;
    std::map<int, PosePtr> _poses;
    std::map<int, std::vector<LandmarkPtr>> _landmarks;
    std::map<int, LandmarkPtr> _unique_landmarks;
    std::map<int, MotionPtr> _pose_measurements;
    std::map<int, std::shared_ptr<std::vector<PerceptionPtr>>> _landmark_measurements;
    int _last_id;
};

#endif // GRAPH_H
