#include <cmath>
#include <iostream>
#include <vector>

#include <ceres/ceres.h>

#include "../include/perceptionenumerator.h"


struct LandmarkErrorFunction
{
    template <typename T>
    bool operator()(const T* const pose, const T* const landmark, T* residual) const
    {
        Eigen::Matrix2<T> rotation;
        rotation(0, 0) = cos(pose[2]);
        rotation(0, 1) = -sin(pose[2]);
        rotation(1, 0) = sin(pose[2]);
        rotation(1, 1) = cos(pose[2]);

        Eigen::Vector2<T> temp;
        temp(0) = landmark[0] - pose[0];
        temp(1) = landmark[1] - pose[1];

        temp = rotation.transpose() * temp;

        residual[0] = temp(0) - landmark[0];
        residual[1] = temp(1) - landmark[1];

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
    double init_landmark[] = {1, 0};

    double pose[][3] = {{0, 0, 0}, {1, 0, 0}};
    double landmark[][2] = {{1, 0}, {2, 0}};

    // Build the problem.
    ceres::Problem problem;

    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<LandmarkErrorFunction, 1, 2, 2>(new LandmarkErrorFunction);
    problem.AddResidualBlock(cost_function, nullptr, *pose, *landmark);

    // Run the solver!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << std::endl;
    std::cout << "p: " << init_pose[0] << " " << init_pose[1] << " " << init_pose[2] << " -> " << pose[0][0] << " " << pose[0][1] << " " << pose[0][2] << std::endl;
    std::cout << "lm: " << init_landmark[0] << " " << init_landmark[1] << " -> " << landmark[0][0] << " " << landmark[0][1] << std::endl;

    return 0;
}
