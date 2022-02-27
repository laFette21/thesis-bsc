#include <iostream>
#include <vector>

#include <ceres/ceres.h>

#include "../include/perceptionenumerator.h"


struct LandmarkErrorFunction
{
    template <typename T>
    bool operator()(const T* const pose, const T* const landmark, T* residual) const
    {
        const T cos_psi = ceres::cos(pose[2]);
        const T sin_psi = ceres::sin(pose[2]);

        Eigen::Matrix2<T> rotation;
        rotation << cos_psi, -sin_psi, sin_psi, cos_psi;

        Eigen::Vector2<T> temp;
        temp(0) = landmark[0] - pose[0];
        temp(1) = landmark[1] - pose[1];

        temp = rotation.transpose() * temp;

        residual[0] = temp(0) - landmark[0];
        residual[1] = temp(1) - landmark[1];

        return true;
    }
};

struct PoseErrorFunction
{
    template <typename T>
    bool operator()(const T* const prev_pose, const T* const curr_pose, T* residual) const
    {
        residual[0] = curr_pose[0] - prev_pose[0];
        residual[1] = curr_pose[1] - prev_pose[1];
        residual[2] = curr_pose[2] - prev_pose[2];

        return true;
    }
};

int main()
{
    /*
    try
    {
        PerceptionEnumerator l_enor("../data/input.txt");

        l_enor.first();
        std::cout << l_enor.current();
        
        while (!l_enor.end())
        {
            l_enor.next();
            std::cout << l_enor.current();
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    */

    double init_pose[] = {0, 0, 0};
    double init_landmark[] = {10, -1};
    double init_landmark2[] = {10, 1};

    double pose[][3] = {{0, 0, 0}, {0.5, 0, 0}, {1, 0, 0}, {1.5, 0, 0}, {2, 0, 0}, {2.5, 0, 0}, {3, 0, 0}, {3.5, 0, 0}, {4, 0, 0}, {4.5, 0, 0}};
    double landmark[][2] = {{10, -1}, {10.1, -1}, {10.2, -1}, {10.3, -1}, {10.4, -1}, {10.5, -1}, {10.6, -1}, {10.7, -1}, {10.8, -1}, {10.9, -1}};
    double landmark2[][2] = {{10, 1}, {10.1, 1}, {10.2, 1}, {10.3, 1}, {10.4, 1}, {10.5, 1}, {10.6, 1}, {10.7, 1}, {10.8, 1}, {10.9, 1}};

    // Build the problem.
    ceres::Problem problem;

    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<LandmarkErrorFunction, 1, 1, 1>(new LandmarkErrorFunction);

    for (size_t i = 0; i < 10; ++i)
    {
        problem.AddResidualBlock(cost_function, nullptr, pose[i], landmark[i]);
        problem.AddResidualBlock(cost_function, nullptr, pose[i], landmark2[i]);
    }

    // Run the solver!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << std::endl;
    std::cout << "p: " << init_pose[0] << " " << init_pose[1] << " " << init_pose[2] << " -> " << pose[0][0] << " " << pose[0][1] << " " << pose[0][2] << std::endl;
    std::cout << "lm1: " << init_landmark[0] << " " << init_landmark[1] << " -> " << landmark[0][0] << " " << landmark[0][1] << std::endl;
    std::cout << "lm2: " << init_landmark2[0] << " " << init_landmark2[1] << " -> " << landmark2[0][0] << " " << landmark2[0][1] << std::endl;

    return 0;
}
