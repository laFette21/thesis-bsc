#include <iostream>
#include <vector>

#include <ceres/ceres.h>

#include "../include/perceptionenumerator.h"


template <typename T>
Eigen::Matrix2<T> RotationMatrix2D(T theta)
{
    Eigen::Matrix2<T> result;

    const T cos_theta = ceres::cos(theta);
    const T sin_theta = ceres::sin(theta);

    result << cos_theta, -sin_theta, sin_theta, cos_theta;

    return result;
}

template <typename T>
Eigen::Vector3<T> t2v(Eigen::Matrix3<T> transformation)
{
    Eigen::Vector3<T> result;

    result[0] = transformation(0, 2);
    result[1] = transformation(1, 2);
    result[2] = ceres::atan2(transformation(1, 0), transformation(0, 0));

    return result;
}

template <typename T>
Eigen::Matrix3<T> v2t(Eigen::Vector3<T> vector)
{
    Eigen::Matrix3<T> result = Eigen::Matrix3<T>::Zero();
    Eigen::Matrix2<T> rotation = RotationMatrix2D<T>(vector[2]);

    result(0, 0) = rotation(0, 0);
    result(0, 1) = rotation(0, 1);
    result(1, 0) = rotation(1, 0);
    result(1, 1) = rotation(1, 1);
    result(0, 2) = vector[0];
    result(1, 2) = vector[1];
    result(2, 2) = 1;

    return result;
}

struct LandmarkErrorFunction
{
    template <typename T>
    bool operator()(const T* const pose, const T* const landmark, const T* const measurement, T* residual) const
    {
        Eigen::Matrix2<T> rotation = RotationMatrix2D<T>(pose[2]);

        Eigen::Vector2<T> temp;
        temp(0) = landmark[0] - pose[0];
        temp(1) = landmark[1] - pose[1];

        temp = rotation.transpose() * temp;

        residual[0] = temp(0) - measurement[0];
        residual[1] = temp(1) - measurement[1];

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

    std::vector<Eigen::Vector3<double>> init_pose = {{0, 0, 0}, {0.5, 0, 0}, {1, 0, 0}};
    std::vector<Eigen::Vector2<double>> init_landmark = {{1, -1}, {1.5, -1}, {2, -1}};
    std::vector<Eigen::Vector2<double>> init_landmark_measurements = {{1, -1}, {1, -1}, {1, -1}};

    std::vector<Eigen::Vector3<double>> pose = {{0, 0, 0}, {0.5, 0, 0}, {1, 0, 0}};
    std::vector<Eigen::Vector2<double>> landmark = {{1, -1}, {1.5, -1}, {2, -1}};
    std::vector<Eigen::Vector2<double>> landmark_measurements = {{1, -1}, {1, -1}, {1, -1}};

    // double init_pose[][3] = {{0, 0, 0}, {0.5, 0, 0}, {1, 0, 0}, {1.5, 0, 0}, {2, 0, 0}, {2.5, 0, 0}, {3, 0, 0}, {3.5, 0, 0}, {4, 0, 0}, {4.5, 0, 0}};
    // double init_landmark[][2] = {{10, -1}, {10.1, -1}, {10.2, -1}, {10.3, -1}, {10.4, -1}, {10.5, -1}, {10.6, -1}, {10.7, -1}, {10.8, -1}, {10.9, -1}};
    // double init_landmark_measurements[][2] = {{10, 1}, {10.1, 1}, {10.2, 1}, {10.3, 1}, {10.4, 1}, {10.5, 1}, {10.6, 1}, {10.7, 1}, {10.8, 1}, {10.9, 1}};

    // double pose[][3] = {{0, 0, 0}, {0.5, 0, 0}, {1, 0, 0}, {1.5, 0, 0}, {2, 0, 0}, {2.5, 0, 0}, {3, 0, 0}, {3.5, 0, 0}, {4, 0, 0}, {4.5, 0, 0}};
    // double landmark[][2] = {{10, -1}, {10.1, -1}, {10.2, -1}, {10.3, -1}, {10.4, -1}, {10.5, -1}, {10.6, -1}, {10.7, -1}, {10.8, -1}, {10.9, -1}};
    // double landmark_measurements[][2] = {{10, 1}, {10.1, 1}, {10.2, 1}, {10.3, 1}, {10.4, 1}, {10.5, 1}, {10.6, 1}, {10.7, 1}, {10.8, 1}, {10.9, 1}};

    // TODO: LOOP for processing perception
        // TODO: LOOP for all the cones visible from the current pose
        // TODO: Solve the problem
        // TODO: REPEAT

    // Build the problem.
    ceres::Problem problem;

    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<LandmarkErrorFunction, 2, 3, 2, 2>(new LandmarkErrorFunction);

    for (size_t i = 0; i < pose.size(); ++i)
    {
        for (size_t j = 0; j < landmark.size(); ++j)
        {
            problem.AddResidualBlock(cost_function, nullptr, pose[i].data(), landmark[j].data(), landmark_measurements[i].data());
            problem.SetParameterBlockConstant(landmark_measurements[i].data());
        }
    }

    problem.SetParameterBlockConstant(pose[0].data());

    // Run the solver!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << std::endl;

    for (size_t i = 0; i < pose.size(); ++i)
    {
        std::cout << i + 1 << ". p: " << init_pose[i][0] << " " << init_pose[i][1] << " " << init_pose[i][2] << " -> " << pose[i][0] << " " << pose[i][1] << " " << pose[i][2] << std::endl;
        std::cout << i + 1 <<  ". lm: " << init_landmark[i][0] << " " << init_landmark[i][1] << " -> " << landmark[i][0] << " " << landmark[i][1] << std::endl;
        std::cout << i + 1 <<  ". meas: " << init_landmark_measurements[i][0] << " " << init_landmark_measurements[i][1] << " -> " << landmark_measurements[i][0] << " " << landmark_measurements[i][1] << std::endl;
    }

    return 0;
}
