#include <iostream>
#include <vector>

#include <ceres/ceres.h>

#include "DataEnumerator.h"
#include "Graph.h"
#include "matplotlibcpp.h"
#include "argparse.hpp"

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
        bool flag = true; // Perception containing id from first_lm_ids was found.
        std::set<int> first_lm_ids;
        int i = 1;

        enor.first();

        if (!enable_loop_closure)
            first_lm_ids = Data::getLandmarkIdsFromPerceptions(enor.current().perceptions);

        while (!enor.end())
        {
            // We need to skip the first N batch of perceptions because loop closure
            // checks if the perception contains an id from the first_lm_ids set
            // which would be irrelevant for the first N batch of perceptions.
            if (!enable_loop_closure && flag)
            {
                std::set<int> current_perceptions = Data::getLandmarkIdsFromPerceptions(enor.current().perceptions);
                bool id_was_in_first_lm_ids = false;

                for (const auto& lm_id : first_lm_ids)
                    id_was_in_first_lm_ids |= current_perceptions.contains(lm_id);

                if (!id_was_in_first_lm_ids)
                    flag = false;
            }
            else if (!enable_loop_closure && !flag)
            {
                std::set<int> current_perceptions = Data::getLandmarkIdsFromPerceptions(enor.current().perceptions);
                bool id_was_in_first_lm_ids = false;

                for (const auto& lm_id : first_lm_ids)
                    id_was_in_first_lm_ids |= current_perceptions.contains(lm_id);

                if (id_was_in_first_lm_ids)
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
