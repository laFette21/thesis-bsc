#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "DataEnumerator.h"

TEST_CASE("Test Data", "[dataenumerator]")
{
    SECTION("Constructing a Data object without arguments")
    {
        const Data data;

        REQUIRE(data == Data());
    }

    SECTION("Constructing a Data object with arguments")
    {
        const Motion motion(1, 2);
        const std::vector<Perception> perceptions{Perception(1, 2, 3, 4, 5, 6)};
        const Data data(motion, perceptions);

        REQUIRE(data == Data(motion, perceptions));
    }

    SECTION("Checking Landmark ids with empty Perception vector")
    {
        const std::vector<Perception> perceptions;

        REQUIRE(Data::getLandmarkIdsFromPerceptions(perceptions) == std::set<int>());
    }

    SECTION("Checking Landmark ids with non-empty Perception vector")
    {
        const std::vector<Perception> perceptions{
            Perception(1, 2, 3, 4, 5, 6),
            Perception(7, 8, 9, 10, 11, 12)
        };

        REQUIRE(Data::getLandmarkIdsFromPerceptions(perceptions) == std::set<int>{1, 7});
    }
}

TEST_CASE("Test DataEnumerator", "[dataenumerator]")
{
    SECTION("Constructing a Data object with non-existing file")
    {
        REQUIRE_THROWS(DataEnumerator("non-existing-file"));
    }

    SECTION("Constructing a Data object with existing empty file")
    {
        REQUIRE_NOTHROW(DataEnumerator("/Users/lafette21/Documents/ELTE/THESIS/thesis/slam_cpp/build/src/dataenumerator/test/empty.txt"));

        DataEnumerator enor("/Users/lafette21/Documents/ELTE/THESIS/thesis/slam_cpp/build/src/dataenumerator/test/empty.txt");

        REQUIRE_FALSE(enor.end()); 

        enor.first();

        REQUIRE(enor.current() == Data());
        REQUIRE(enor.end());
    }
}
