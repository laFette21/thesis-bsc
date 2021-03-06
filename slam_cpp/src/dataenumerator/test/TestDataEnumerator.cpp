#define CATCH_CONFIG_MAIN

#include <filesystem>

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
    SECTION("Constructing a DataEnumerator object with non-existing file")
    {
        REQUIRE_THROWS(DataEnumerator("non-existing-file", 0, false));
    }

    SECTION("Constructing a DataEnumerator object with existing empty file")
    {
        REQUIRE_NOTHROW(DataEnumerator("empty.txt", 0, false));
    }

    SECTION("Reading from an empty file")
    {
        DataEnumerator enor("empty.txt", 0, false);

        REQUIRE_FALSE(enor.end()); 

        enor.first();

        REQUIRE(enor.current() == Data());
        REQUIRE(enor.end());
    }

    SECTION("Noise is 0")
    {
        SECTION("Reading a Data(1 Motion, 2 Perceptions) from a non-empty file")
        {
            DataEnumerator enor("input1.txt", 0, false);

            REQUIRE_FALSE(enor.end()); 

            enor.first();

            REQUIRE(enor.current() == Data(Motion(5, 0.1), {Perception(0, 0, 0, 1, 5, 2), Perception(1, 0, 0, 2, 5, -2)}));
            REQUIRE(enor.end());
        }

        SECTION("Reading 2 Data(1 Motion, 2 Perceptions) from a non-empty file")
        {
            DataEnumerator enor("input2.txt", 0, false);

            REQUIRE_FALSE(enor.end()); 

            enor.first();

            REQUIRE(enor.current() == Data(Motion(5, 0.1), {Perception(0, 0, 0, 1, 5, 2), Perception(1, 0, 0, 2, 5, -2)}));
            REQUIRE_FALSE(enor.end());

            enor.next();

            REQUIRE(enor.current() == Data(Motion(5.5, 0.1), {Perception(0, 0, 0, 1, 5, 2), Perception(1, 0, 0, 2, 5, -2)}));
            REQUIRE(enor.end());
        }

        SECTION("Reading only Motion data from a non-empty file")
        {
            DataEnumerator enor("input3.txt", 0, false);

            REQUIRE_FALSE(enor.end()); 

            enor.first();

            REQUIRE(enor.current() == Data(Motion(4.8, 0.1), {}));
            REQUIRE(enor.end());
        }
    }

    SECTION("Noise is non-zero")
    {
        SECTION("Random noise")
        {
            DataEnumerator enor1("input1.txt", 0.1, true);
            DataEnumerator enor2("input1.txt", 0.1, true);

            enor1.first();
            enor2.first();

            REQUIRE_FALSE(enor1.current() == enor2.current());
        }

        SECTION("Pseudo-random noise")
        {
            DataEnumerator enor1("input1.txt", 0.1, false);
            DataEnumerator enor2("input1.txt", 0.1, false);

            enor1.first();
            enor2.first();

            REQUIRE(enor1.current() == enor2.current());
        }

        SECTION("Reading a Data(1 Motion, 2 Perceptions) from a non-empty file")
        {
            DataEnumerator enor("input1.txt", 0.1, false);

            REQUIRE_FALSE(enor.end()); 

            enor.first();

            REQUIRE_FALSE(enor.current() == Data(Motion(5, 0.1), {Perception(0, 0, 0, 1, 5, 2), Perception(1, 0, 0, 2, 5, -2)}));
            REQUIRE(enor.end());
        }

        SECTION("Reading 2 Data(1 Motion, 2 Perceptions) from a non-empty file")
        {
            DataEnumerator enor("input2.txt", 0.1, false);

            REQUIRE_FALSE(enor.end()); 

            enor.first();

            REQUIRE_FALSE(enor.current() == Data(Motion(5, 0.1), {Perception(0, 0, 0, 1, 5, 2), Perception(1, 0, 0, 2, 5, -2)}));
            REQUIRE_FALSE(enor.end());

            enor.next();

            REQUIRE_FALSE(enor.current() == Data(Motion(5.5, 0.1), {Perception(0, 0, 0, 1, 5, 2), Perception(1, 0, 0, 2, 5, -2)}));
            REQUIRE(enor.end());
        }

        SECTION("Reading only Motion data from a non-empty file")
        {
            DataEnumerator enor("input3.txt", 0.1, false);

            REQUIRE_FALSE(enor.end()); 

            enor.first();

            REQUIRE_FALSE(enor.current() == Data(Motion(4.8, 0.1), {}));
            REQUIRE(enor.end());
        }
    }
}
