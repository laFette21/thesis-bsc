#include "catch.hpp"
#include "Utils.h"

TEST_CASE("Test RotationMatrix2D", "[utility]")
{
    SECTION("Identity matrix with theta = 0")
    {
        REQUIRE(Eigen::Matrix2d::Identity() == RotationMatrix2D<double>(0));
    }

    SECTION("Matrix with theta from range [0.01, 0.02..1]")
    {
        Eigen::Matrix2d matrix;

        for (size_t i = 1; i <= 100; ++i)
        {
            const double sin_theta = sin(i * 0.01);
            const double cos_theta = cos(i * 0.01);

            matrix << cos_theta, -sin_theta, sin_theta, cos_theta;

            REQUIRE(matrix == RotationMatrix2D<double>(i * 0.01));
        }
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
