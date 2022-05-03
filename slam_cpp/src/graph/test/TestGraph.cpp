#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "Graph.h"

TEST_CASE("Test Graph", "[graph]")
{
    SECTION("Constructing a Graph object")
    {
        const Graph graph;

        REQUIRE(graph.getLastId() == 1);
        REQUIRE(graph.getPoses().size() == 1);
        REQUIRE(graph.getUniqueLandmarks().size() == 0);
        REQUIRE(*graph.getPoses().begin()->second == Pose());
    }

    SECTION("Creating a Landmark")
    {
        Graph graph;
        std::shared_ptr<std::vector<std::shared_ptr<Perception>>> perceptions(new std::vector<std::shared_ptr<Perception>>());
        perceptions->push_back(std::shared_ptr<Perception>(new Perception(0, 0, 0, 0, 0, 0)));
        graph.createLandmark(perceptions);

        REQUIRE(graph.getUniqueLandmarks().size() == 1);

        graph.createLandmark(perceptions);
        REQUIRE(graph.getUniqueLandmarks().size() == 1);
    }

    SECTION("Creating a Pose with Motion(0, 0)")
    {
        Graph graph;
        const auto motion = std::make_shared<Motion>(0, 0);
        graph.createPose(motion);

        REQUIRE(graph.getLastId() == 2);
        REQUIRE(graph.getPoses().size() == 2);
        REQUIRE(graph.getUniqueLandmarks().size() == 0);
        REQUIRE(*graph.getPoses().begin()->second == Pose());
        REQUIRE(*graph.getPoses().rbegin()->second == Pose());
    }

    SECTION("Creating a Pose with Motion(1, 1)")
    {
        Graph graph;
        const auto motion = std::make_shared<Motion>(1, 1);
        graph.createPose(motion);

        REQUIRE(graph.getLastId() == 2);
        REQUIRE(graph.getPoses().size() == 2);
        REQUIRE(graph.getUniqueLandmarks().size() == 0);
        REQUIRE(*graph.getPoses().begin()->second == Pose());
        REQUIRE_FALSE(*graph.getPoses().rbegin()->second == Pose());
    }

    SECTION("Optimizing the Graph")
    {
        Graph graph;
        graph.createPose(std::make_shared<Motion>(1, 1));

        std::shared_ptr<std::vector<std::shared_ptr<Perception>>> perceptions(new std::vector<std::shared_ptr<Perception>>());
        perceptions->push_back(std::shared_ptr<Perception>(new Perception(0, 3, 2, 0, 0, 0)));
        perceptions->push_back(std::shared_ptr<Perception>(new Perception(1, 3, -2, 0, 0, 0)));
        graph.createLandmark(perceptions);

        graph.createPose(std::make_shared<Motion>(1, 0));

        perceptions->clear();
        perceptions->push_back(std::shared_ptr<Perception>(new Perception(0, 4, 1, 0, 0, 0)));
        perceptions->push_back(std::shared_ptr<Perception>(new Perception(1, 4, -1, 0, 0, 0)));

        graph.createLandmark(perceptions);

        auto poses = graph.getPoses();
        auto lms = graph.getUniqueLandmarks();

        const auto p1_pre = *poses[1];
        const auto p2_pre = *poses[2];
        const auto lm1_pre = *lms.begin()->second;
        const auto lm2_pre = *lms.rbegin()->second;

        REQUIRE(graph.optimize(-1, true));

        const auto p1_post = *poses[1];
        const auto p2_post = *poses[2];
        const auto lm1_post = *lms.begin()->second;
        const auto lm2_post = *lms.rbegin()->second;

        REQUIRE_FALSE(p1_pre == p1_post);
        REQUIRE_FALSE(p2_pre == p2_post);
        REQUIRE_FALSE(lm1_pre == lm1_post);
        REQUIRE_FALSE(lm2_pre == lm2_post);
    }
}
