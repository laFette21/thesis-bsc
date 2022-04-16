#include "catch.hpp"
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
/*
TEST_CASE("Test LandmarkErrorFunction", "[utility]")
{
    SECTION("")
    {

    }
}

TEST_CASE("Test PoseErrorFunction", "[utility]")
{
    SECTION("")
    {
        constexpr int size = 100;
        std::vector<double> measurements(size);

        for (size_t i = 0; i < size; ++i)
        {
            measurements[i] = i * 0.01;
        }
    }
}
*/
