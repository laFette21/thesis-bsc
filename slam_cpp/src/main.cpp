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
Eigen::Vector3<T> t2v(const Eigen::Matrix3<T>& transformation)
{
    Eigen::Vector3<T> result;

    result[0] = transformation(0, 2);
    result[1] = transformation(1, 2);
    result[2] = ceres::atan2(transformation(1, 0), transformation(0, 0));

    return result;
}

template <typename T>
Eigen::Matrix3<T> v2t(const Eigen::Vector3<T>& vector)
{
    Eigen::Matrix3<T> result = Eigen::Matrix3<T>::Zero();

    result.template topLeftCorner<2, 2>() = RotationMatrix2D<T>(vector[2]);
    result.template topRightCorner<2, 1>() = vector.template head<2>();
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
    bool operator()(const T* const prev_pose, const T* const curr_pose, const T* const measurement, T* residual) const
    {
        Eigen::Matrix3<T> prev_pose_transformation = v2t(Eigen::Vector3<T>(prev_pose));
        Eigen::Matrix3<T> curr_pose_transformation = v2t(Eigen::Vector3<T>(curr_pose));
        Eigen::Matrix3<T> measurement_transformation = v2t(Eigen::Vector3<T>(measurement));
        Eigen::Vector3<T> vec = t2v((curr_pose_transformation.inverse() * prev_pose_transformation) * measurement_transformation);

        residual[0] = vec[0];
        residual[1] = vec[1];
        residual[2] = vec[2];

        return true;
    }
};

#define NORMAL_MODE
#ifdef NORMAL_MODE

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

    Eigen::Matrix3<double> prev_pose_transformation = v2t(Eigen::Vector3<double>(0, 0, 0));
    std::cout << prev_pose_transformation << std::endl << std::endl;
    Eigen::Matrix3<double> curr_pose_transformation = v2t(Eigen::Vector3<double>(1, 1, 1));
    std::cout << curr_pose_transformation << std::endl << std::endl;
    Eigen::Matrix3<double> measurement_transformation = v2t(Eigen::Vector3<double>(1, 1, 1));
    std::cout << measurement_transformation << std::endl << std::endl;
    Eigen::Vector3<double> vec = t2v<double>((curr_pose_transformation.inverse() * prev_pose_transformation) * measurement_transformation);
    std::cout << (curr_pose_transformation.inverse() * prev_pose_transformation) * measurement_transformation << std::endl << std::endl;
    std::cout << vec << std::endl << std::endl;

    return 0;
}

#else

#define CATCH_CONFIG_MAIN
#include "../include/catch.hpp"


TEST_CASE("Test RotationMatrix2D", "[utility]")
{
    SECTION("Identity matrix with theta = 0")
    {
        REQUIRE(Eigen::Matrix2d::Identity() == RotationMatrix2D<double>(0));
    }

    SECTION("Matrix with theta = 1")
    {
        Eigen::Matrix2d matrix;

        const double sin_theta = sin(1);
        const double cos_theta = cos(1);

        matrix << cos_theta, -sin_theta, sin_theta, cos_theta;

        REQUIRE(matrix == RotationMatrix2D<double>(1));
    }
}

TEST_CASE("Test t2v", "[utility]")
{
    SECTION("Vector of zeros with identity matrix")
    {
        REQUIRE(Eigen::Vector3d::Zero() == t2v<double>(Eigen::Matrix3d::Identity()));
    }

    SECTION("Vector of ones with transformation matrix")
    {
        Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();

        matrix.topLeftCorner(2, 2) = RotationMatrix2D<double>(1);
        matrix.topRightCorner(2, 1) = Eigen::Vector2<double>::Ones();
        matrix(2, 2) = 1;

        REQUIRE(Eigen::Vector3d::Ones() == t2v<double>(matrix));
    }
}

TEST_CASE("Test v2t", "[utility]")
{
    SECTION("Identity matrix with vector of zeros")
    {
        REQUIRE(Eigen::Matrix3d::Identity() == v2t<double>(Eigen::Vector3d::Zero()));
    }

    SECTION("Transformation matrix with vector of ones")
    {
        Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();

        matrix.topLeftCorner(2, 2) = RotationMatrix2D<double>(1);
        matrix.topRightCorner(2, 1) = Eigen::Vector2<double>::Ones();
        matrix(2, 2) = 1;

        REQUIRE(matrix == v2t<double>(Eigen::Vector3d::Ones()));
    }
}

#endif
