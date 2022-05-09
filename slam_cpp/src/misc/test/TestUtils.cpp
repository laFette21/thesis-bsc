#include "catch.hpp"
#include "Types.h"
#include "Utils.h"

TEST_CASE("Test RotationMatrix2D", "[utility]")
{
    SECTION("Identity matrix with theta = 0")
    {
        REQUIRE(Eigen::Matrix2d::Identity() == RotationMatrix2D<double>(0));
    }

    SECTION("Matrix with theta = π")
    {
        Eigen::Matrix2d matrix;

        const double sin_theta = sin(M_PI);
        const double cos_theta = cos(M_PI);

        matrix << cos_theta, -sin_theta, sin_theta, cos_theta;

        REQUIRE(matrix == RotationMatrix2D<double>(M_PI));
    }

    SECTION("Matrix with theta = -π")
    {
        Eigen::Matrix2d matrix;

        const double sin_theta = sin(-M_PI);
        const double cos_theta = cos(-M_PI);

        matrix << cos_theta, -sin_theta, sin_theta, cos_theta;

        REQUIRE(matrix == RotationMatrix2D<double>(-M_PI));
    }

    SECTION("Matrix with theta = 2π")
    {
        Eigen::Matrix2d matrix;

        const double sin_theta = sin(2 * M_PI);
        const double cos_theta = cos(2 * M_PI);

        matrix << cos_theta, -sin_theta, sin_theta, cos_theta;

        REQUIRE(matrix == RotationMatrix2D<double>(2 * M_PI));
    }

    SECTION("Matrix with theta = -2π")
    {
        Eigen::Matrix2d matrix;

        const double sin_theta = sin(-2 * M_PI);
        const double cos_theta = cos(-2 * M_PI);

        matrix << cos_theta, -sin_theta, sin_theta, cos_theta;

        REQUIRE(matrix == RotationMatrix2D<double>(-2 * M_PI));
    }
}

TEST_CASE("Test LandmarkErrorFunction", "[utility]")
{
    SECTION("Perceiving landmarks from the Pose(0, 0, 0) in a semicircle")
    {
        constexpr int size = 4;
        const double sqrt2_2 = sqrt(2) / 2;
        double error = 0;
        Eigen::Vector3d pose{0, 0, 0};
        std::vector<Eigen::Vector2d> residuals(size, {0, 0});
        std::vector<Eigen::Vector2d> landmarks{
            {0, 1},
            {sqrt2_2, sqrt2_2},
            {1, 0},
            {sqrt2_2, -sqrt2_2},
            {0, -1}
        };
        std::vector<Eigen::Vector2d> measurements{
            {1, M_PI_2},
            {1, M_PI_4},
            {1, 0},
            {1, -M_PI_4},
            {1, -M_PI_2}
        };

        for (size_t i = 0; i < size; ++i)
            LandmarkErrorFunction()(pose.data(), landmarks[i].data(), measurements[i].data(), residuals[i].data());

        for (size_t i = 0; i < size; ++i)
            error += residuals[i](0) + residuals[i](1);

        REQUIRE(error == Approx(0).margin(1e-12));
    }
}

TEST_CASE("Test PoseErrorFunction", "[utility]")
{
    SECTION("Straight line with constant 1 m/s velocity and 0 rad/s angular velocity")
    {
        constexpr int size = 10;
        constexpr double measurement[2] = {1, 0};
        double error = 0;
        std::vector<Eigen::Vector3d> poses(size, {0, 0, 0});
        std::vector<Eigen::Vector3d> residuals(size - 1, {0, 0, 0});

        for (size_t i = 0; i < size; ++i)
            poses[i](0) = i;

        for (size_t i = 1; i < size; ++i)
            PoseErrorFunction()(poses[i - 1].data(), poses[i].data(), measurement, residuals[i - 1].data());
        
        for (size_t i = 0; i < size - 1; ++i)
            error += residuals[i](0) + residuals[i](1) + residuals[i](2);

        REQUIRE(error == Approx(0).margin(1e-12));
    }

    SECTION("Oval line with constant 10 m/s velocity and variable angular velocity")
    {
        double error = 0;
        std::vector<Motion> measurements{
            {1, 0},
            {1, M_PI_2},
            {1, M_PI_2},
            {1, 0}
        };
        std::vector<Pose> poses{Pose()};
        std::vector<Eigen::Vector3d> residuals(measurements.size(), {0, 0, 0});

        for (size_t i = 0; i < measurements.size(); ++i)
        {
            Pose pose(poses.back());
            poses.emplace_back(pose += measurements[i]);
        }

        for (size_t i = 1; i < poses.size(); ++i)
        {
            measurements[i - 1].data[0] *= 0.05;
            measurements[i - 1].data[1] *= 0.025;

            PoseErrorFunction()(poses[i - 1].data, poses[i].data, measurements[i - 1].data, residuals[i - 1].data());
        }

        for (size_t i = 0; i < residuals.size(); ++i)
            error += residuals[i](0) + residuals[i](1) + residuals[i](2);

        REQUIRE(error == Approx(0).margin(1e-12));
    }
}
