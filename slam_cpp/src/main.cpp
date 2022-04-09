#include <iostream>
#include <vector>
#include <chrono>

#include <ceres/ceres.h>

#include "../include/dataenumerator.h"
#include "../include/graph.h"

#define NORMAL_MODE
#ifdef NORMAL_MODE

static constexpr int section = (int)(5 / TIMESTAMP);
static constexpr bool enable_loop_closure = true;

int main(int argc, char const *argv[])
{
    auto start = std::chrono::steady_clock::now();

    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <path/to/input/file>" << " <path/to/output/file>" << std::endl;
        return 1;
    }

    try
    {
        std::ofstream output(argv[2]);
        DataEnumerator enor(argv[1]);
        Graph graph;
        bool flag = true;
        std::set<int> first_lm_ids;
        int i = 1;

        enor.first();

        if (!enable_loop_closure)
            first_lm_ids = Data::getLandmarkIdsFromPerceptions(enor.current().perceptions);

        while (!enor.end())
        {
            if (!enable_loop_closure && flag)
            {
                std::set<int> current_perceptions = Data::getLandmarkIdsFromPerceptions(enor.current().perceptions);
                bool b = false;

                for (auto& lm_id : first_lm_ids)
                    b |= current_perceptions.contains(lm_id);

                if (!b)
                    flag = false;
            }
            else if (!enable_loop_closure && !flag)
            {
                std::set<int> current_perceptions = Data::getLandmarkIdsFromPerceptions(enor.current().perceptions);
                bool b = false;

                for (auto& lm_id : first_lm_ids)
                    b |= current_perceptions.contains(lm_id);

                if (b)
                    break;
            }

            std::shared_ptr<std::vector<std::shared_ptr<Perception>>> perceptions(new std::vector<std::shared_ptr<Perception>>());

            for (auto& perception : enor.current().perceptions)
                perceptions->push_back(std::shared_ptr<Perception>(new Perception(perception)));

            graph.createLandmark(perceptions);

            std::shared_ptr<Motion> motion(new Motion(enor.current().motion));
            graph.createPose(motion);
/*
            if (i % section == 0)
            {
                output << "===" << std::endl;
                output << graph;
                output << "###" << std::endl;

                auto lms = graph.getUniqueLandmarks();

                for (const auto& lm : lms)
                {
                    output << *lm.second << std::endl;
                }

                output << "###" << std::endl;

                graph.optimize();

                output << graph;
                output << "###" << std::endl;

                lms = graph.getUniqueLandmarks();

                for (const auto& lm : lms)
                {
                    output << *lm.second << std::endl;
                }
            }
*/
            i++;
            enor.next();
        }

        output << "===" << std::endl;
        output << graph;
        output << "###" << std::endl;

        auto lms = graph.getUniqueLandmarks();

        for (const auto& lm : lms)
        {
            output << *lm.second << std::endl;
        }

        output << "###" << std::endl;

        graph.optimize(-1, true);

        output << graph;
        output << "###" << std::endl;

        lms = graph.getUniqueLandmarks();

        for (const auto& lm : lms)
        {
            output << *lm.second << std::endl;
        }

        output << "===" << std::endl;
        output << "END" << std::endl;

        output.close();
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

#define EPSILON 0.00001

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

TEST_CASE("Test PoseErrorFunction", "[utility]")
{
    SECTION("Input data without noise")
    {
        DataEnumerator enor("../data/input_kecso_noisy.txt");
        std::vector<Motion> measurements;

        enor.first();

        while (!enor.end())
        {
            measurements.emplace_back(enor.current().motion);

            enor.next();
        }

        std::vector<Pose> poses{Pose()};

        for (int i = 0; i < measurements.size(); ++i)
            poses.emplace_back(poses[i] += measurements[i]);

        std::cerr << poses.size() << std::endl;

        std::vector<Eigen::Vector3d> residuals(poses.size(), Eigen::Vector3d());

        for (int i = 0; i < poses.size(); ++i)
        {
            measurements[i].data[0] *= 0.05;
            measurements[i].data[1] *= 0.025;

            PoseErrorFunction()(poses[i].data, poses[i + 1].data, measurements[i].data, residuals[i].data());
        }

        double error_x = 0, error_y = 0, error_psi = 0;

        for (int i = 0; i < 99; ++i)
        {
            error_x += residuals[i](0);
            error_y += residuals[i](1);
            error_psi += residuals[i](2);
        }

        std::cerr << error_x << " " << error_y << " " << error_psi << std::endl;

        REQUIRE(error_x <= EPSILON);
        REQUIRE(error_y <= EPSILON);
        REQUIRE(error_psi <= EPSILON);
    }
}

#endif
