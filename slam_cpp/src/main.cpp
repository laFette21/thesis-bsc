#include <iostream>
#include <vector>
#include <chrono>

#include <ceres/ceres.h>

#include "../include/dataenumerator.h"
#include "../include/graph.h"


#define NORMAL_MODE
#ifdef NORMAL_MODE

int main()
{
    auto start = std::chrono::steady_clock::now();

    try
    {
        Graph graph;
        DataEnumerator enor("../data/input_noisy.txt");
        int i = 1;

        enor.first();
        // std::cout << l_enor.current();

        while (!enor.end())
        {
            // if (i == 100) break;
            // std::cout << enor.current();

            std::shared_ptr<std::vector<std::shared_ptr<Perception>>> perceptions(new std::vector<std::shared_ptr<Perception>>());

            for (auto& perception : enor.current().perceptions)
                perceptions->push_back(std::shared_ptr<Perception>(new Perception(perception)));

            graph.createLandmark(perceptions);

            std::shared_ptr<Odometry> odometry(new Odometry(enor.current().odometry));
            graph.createPose(odometry);

            // if (i % 30 == 0)
            //     graph.optimize(true);

            std::cerr << i++ << std::endl;

            enor.next();
        }

        graph.optimize(true);

        std::cout << graph;

        auto all_lms = graph.getLandmarks();

        for (auto& lms : all_lms)
        {
            for (auto& lm : lms.second)
            {
                if (lm->id == 247)
                {
                    std::cerr << *lm << std::endl;
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    auto end = std::chrono::steady_clock::now();

    std::cerr << "Elapsed time in milliseconds: "
        << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
        << " ms" << std::endl;

/*
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
*/
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
