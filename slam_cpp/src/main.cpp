#include <iostream>
#include <vector>
#include <chrono>

#include <ceres/ceres.h>

#include "../include/dataenumerator.h"
#include "../include/graph.h"

#define NORMAL_MODE
#ifdef NORMAL_MODE

int main(int argc, char const *argv[])
{
    auto start = std::chrono::steady_clock::now();

    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <path/to/file>" << std::endl;
        return 1;
    }

    try
    {
        Graph graph;
        DataEnumerator enor(argv[1]);
        int i = 1;

        enor.first();

        auto is_id = [](std::shared_ptr<Perception> obj){ return obj->id == 247; };

        while (!enor.end())
        {
            std::shared_ptr<std::vector<std::shared_ptr<Perception>>> perceptions(new std::vector<std::shared_ptr<Perception>>());

            for (auto& perception : enor.current().perceptions)
                perceptions->push_back(std::shared_ptr<Perception>(new Perception(perception)));

            // if (std::find_if(perceptions->begin(), perceptions->end(), is_id) == perceptions->end())
            //     break;

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

        for (const auto& lms : all_lms)
        {
            for (const auto& lm : lms.second)
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
