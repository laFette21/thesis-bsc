#include <iostream>
#include <vector>
#include <chrono>

#include <ceres/ceres.h>

#include "DataEnumerator.h"
#include "Graph.h"
#include "matplotlibcpp.h"
#include "argparse.hpp"

#define NORMAL_MODE
#ifdef NORMAL_MODE

static constexpr int section = (int)(5 / timestamp);

int main(int argc, char const *argv[])
{
    argparse::ArgumentParser program("slam");

    program.add_argument("input_file")
        .help("path to the input file");
    program.add_argument("output_file")
        .help("path where the output file should be saved");
    program.add_argument("-l", "--loop_closure")
        .help("enables loop closing for the optimization")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("-n", "--noise")
        .help("uses noise on the input data")
        .default_value(0.0)
        .scan<'g', double>();
    program.add_argument("-p", "--plot")
        .help("plots the built map and the poses using matplotlib")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("-r", "--random")
        .help("uses random seed for noise generation")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("-s", "--segmentation")
        .help("enables the segmentation of the data")
        .default_value(false)
        .implicit_value(true);

    try
    {
        program.parse_args(argc, argv);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << program;
        std::exit(1);
    }

    try
    {
        std::ofstream output(program.get<std::string>("output_file"));
        DataEnumerator enor(
            program.get<std::string>("input_file"),
            program.get<double>("--noise"),
            program.get<bool>("--random")
        );
        Graph graph;
        std::vector<double> lm_x_gt, lm_y_gt, lm_x_pre, lm_y_pre, lm_x_post, lm_y_post;
        std::vector<double> p_x_pre, p_y_pre, p_x_post, p_y_post;
        std::vector<std::vector<double>> lm_vec_x, lm_vec_y;
        bool enable_loop_closure = program.get<bool>("--loop_closure");
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

                for (const auto& lm_id : first_lm_ids)
                    b |= current_perceptions.contains(lm_id);

                if (!b)
                    flag = false;
            }
            else if (!enable_loop_closure && !flag)
            {
                std::set<int> current_perceptions = Data::getLandmarkIdsFromPerceptions(enor.current().perceptions);
                bool b = false;

                for (const auto& lm_id : first_lm_ids)
                    b |= current_perceptions.contains(lm_id);

                if (b)
                    break;
            }

            std::shared_ptr<std::vector<std::shared_ptr<Perception>>> perceptions(new std::vector<std::shared_ptr<Perception>>());

            for (const auto& perception : enor.current().perceptions)
                perceptions->push_back(std::shared_ptr<Perception>(new Perception(perception)));

            graph.createLandmark(perceptions);

            std::shared_ptr<Motion> motion(new Motion(enor.current().motion));
            graph.createPose(motion);

            if (program.get<bool>("--segmentation") && i % section == 0)
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

                if (program.get<bool>("--plot"))
                {
                    matplotlibcpp::xlabel("x[m]");
                    matplotlibcpp::ylabel("y[m]");
                    matplotlibcpp::title("Map");
                    matplotlibcpp::axis("equal");

                    for (const auto& lm : lms)
                    {
                        lm_x_pre.push_back(lm.second->data[0]);
                        lm_y_pre.push_back(lm.second->data[1]);
                    }
                }

                graph.optimize();

                output << graph;
                output << "###" << std::endl;

                lms = graph.getUniqueLandmarks();

                for (const auto& lm : lms)
                {
                    output << *lm.second << std::endl;
                }

                if (program.get<bool>("--plot"))
                {
                    for (const auto& lm : lms)
                    {
                        lm_x_post.push_back(lm.second->data[0]);
                        lm_y_post.push_back(lm.second->data[1]);
                    }

                    for (size_t i = 0; i < lm_x_post.size(); ++i)
                    {
                        lm_vec_x.push_back({lm_x_pre[i], lm_x_post[i]});
                        lm_vec_y.push_back({lm_y_pre[i], lm_y_post[i]});
                    }

                    for (size_t i = 0; i < lm_vec_x.size(); ++i)
                    {
                        matplotlibcpp::plot(lm_vec_x[i], lm_vec_y[i], "tab:gray");
                    }

                    matplotlibcpp::named_plot("landmark before optimization", lm_x_pre, lm_y_pre, "g.");
                    matplotlibcpp::named_plot("landmark after optimization", lm_x_post, lm_y_post, "r.");

                    matplotlibcpp::legend();
                    matplotlibcpp::show();
                }
            }

            lm_x_pre.clear();
            lm_y_pre.clear();
            lm_x_post.clear();
            lm_y_post.clear();
            lm_vec_x.clear();
            lm_vec_y.clear();

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

        if (program.get<bool>("--plot"))
        {
            matplotlibcpp::xlabel("x[m]");
            matplotlibcpp::ylabel("y[m]");
            matplotlibcpp::title("Map and trajectory");
            matplotlibcpp::axis("equal");

            auto poses = graph.getPoses();

            for (const auto& pose : poses)
            {
                p_x_pre.push_back(pose.second->data[0]);
                p_y_pre.push_back(pose.second->data[1]);
            }

            for (const auto& lm : lms)
            {
                lm_x_gt.push_back(lm.second->ground_truth[0]);
                lm_y_gt.push_back(lm.second->ground_truth[1]);
                lm_x_pre.push_back(lm.second->data[0]);
                lm_y_pre.push_back(lm.second->data[1]);
            }
        }

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

        if (program.get<bool>("--plot"))
        {
            const auto poses = graph.getPoses();

            for (const auto& pose : poses)
            {
                p_x_post.push_back(pose.second->data[0]);
                p_y_post.push_back(pose.second->data[1]);
            }

            for (const auto& lm : lms)
            {
                lm_x_post.push_back(lm.second->data[0]);
                lm_y_post.push_back(lm.second->data[1]);
            }

            for (size_t i = 0; i < lm_x_post.size(); ++i)
            {
                lm_vec_x.push_back({lm_x_pre[i], lm_x_post[i]});
                lm_vec_y.push_back({lm_y_pre[i], lm_y_post[i]});
            }

            for (size_t i = 0; i < lm_vec_x.size(); ++i)
            {
                matplotlibcpp::plot(lm_vec_x[i], lm_vec_y[i], "tab:gray");
            }

            matplotlibcpp::named_plot("pose before optimization", p_x_pre, p_y_pre, "y.");
            matplotlibcpp::named_plot("pose after optimization", p_x_post, p_y_post, "k.");
            matplotlibcpp::named_plot("landmark ground truth", lm_x_gt, lm_y_gt, "b.");
            matplotlibcpp::named_plot("landmark before optimization", lm_x_pre, lm_y_pre, "g.");
            matplotlibcpp::named_plot("landmark after optimization", lm_x_post, lm_y_post, "r.");

            matplotlibcpp::legend();
            matplotlibcpp::show();
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

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
