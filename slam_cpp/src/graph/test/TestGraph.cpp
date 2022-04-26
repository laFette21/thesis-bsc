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
}
