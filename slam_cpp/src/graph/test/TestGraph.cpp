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

}
