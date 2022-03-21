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

            if (i % (int)(5 / TIMESTAMP) == 0)
                graph.optimize((5 / TIMESTAMP) + 1, true);

            std::cerr << i << std::endl;

            i++;
            enor.next();
        }

        graph.optimize(2001, true);

        std::cout << graph;
        std::cout << "###" << std::endl;

        auto lms = graph.getUniqueLandmarks();

        for (const auto& lm : lms)
        {
            // if (lm->id == 247)
            // {
                std::cout << *lm.second << std::endl;
            // }
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

#endif
