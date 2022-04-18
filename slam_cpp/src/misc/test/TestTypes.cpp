#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "Types.h"

TEST_CASE("Test Landmark", "[types]")
{
    SECTION("Constructing a Landmark object without arguments")
    {
        const Landmark landmark;

        REQUIRE(landmark == Landmark());
    }

    SECTION("Constructing a Landmark object with arguments")
    {
        const Landmark landmark(1, 2, 3, 4, 5, 6);

        REQUIRE(landmark == Landmark(1, 2, 3, 4, 5, 6));
    }
}

TEST_CASE("Test Motion", "[types]")
{
    SECTION("Constructing a Motion object without arguments")
    {
        const Motion motion;

        REQUIRE(motion == Motion());
    }

    SECTION("Constructing a Motion object with arguments")
    {
        const Motion motion(1, 2);

        REQUIRE(motion == Motion(1, 2));
    }
}

TEST_CASE("Test Perception", "[types]")
{
    SECTION("Constructing a Perception object without arguments")
    {
        const Perception perception;

        REQUIRE(perception == Perception());
    }

    SECTION("Constructing a Perception object with arguments")
    {
        const Perception perception(1, 2, 3, 4, 5, 6);

        REQUIRE(perception == Perception(1, 2, 3, 4, 5, 6));
    }
}

TEST_CASE("Test Pose", "[types]")
{
    SECTION("Constructing a Pose object without arguments")
    {
        const Pose pose;

        REQUIRE(pose == Pose());
    }

    SECTION("Constructing a Pose object with arguments")
    {
        const Pose pose(1, 2, 3);

        REQUIRE(pose == Pose(1, 2, 3));
    }

    SECTION("operator+= with default Motion object")
    {
        const Motion motion;
        Pose pose;

        pose += motion;

        REQUIRE(pose == Pose());
    }

    SECTION("operator+= with custom Motion object")
    {
        const Motion motion(1, 1);
        Pose pose;

        pose += motion;

        REQUIRE(pose == Pose(0.05 * cos(0.025), 0.05 * sin(0.025), 0.05));

        pose += motion;

        REQUIRE(pose.data[0] == Approx(0.05 * cos(0.025) + 0.05 * cos(0.075)).margin(1e-12));
        REQUIRE(pose.data[1] == Approx(0.05 * sin(0.025) + 0.05 * sin(0.075)).margin(1e-12));
        REQUIRE(pose.data[2] == Approx(0.1).margin(1e-12));
    }
}
