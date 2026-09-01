#include <gtest/gtest.h>

#include <vector>

#include "pathfinder.hpp"
#include "position.hpp"
#include "warehouse.hpp"


bool positionsEqual(
    Position a,
    Position b
) {
    return a.row == b.row &&
           a.col == b.col;
}


TEST(
    PathfinderTest,
    FindsShortestPathInOpenWarehouse
) {
    Warehouse warehouse(3, 3);

    Position start{0, 0};
    Position goal{2, 2};

    std::vector<Position> path =
        findPath(
            warehouse,
            start,
            goal
        );

    ASSERT_FALSE(
        path.empty()
    );

    EXPECT_EQ(
        path.size(),
        5u
    );

    EXPECT_TRUE(
        positionsEqual(
            path.front(),
            start
        )
    );

    EXPECT_TRUE(
        positionsEqual(
            path.back(),
            goal
        )
    );
}


TEST(
    PathfinderTest,
    AvoidsWarehouseObstacles
) {
    Warehouse warehouse(3, 3);

    warehouse.addObstacle({0, 1});

    Position start{0, 0};
    Position goal{0, 2};

    std::vector<Position> path =
        findPath(
            warehouse,
            start,
            goal
        );

    ASSERT_FALSE(
        path.empty()
    );

    // Direct route would require 2 moves.
    // Obstacle forces:
    //
    // (0,0)
    //   ↓
    // (1,0)
    //   →
    // (1,1)
    //   →
    // (1,2)
    //   ↑
    // (0,2)
    //
    // 4 moves = 5 positions.

    EXPECT_EQ(
        path.size(),
        5u
    );

    for (Position position : path) {
        EXPECT_FALSE(
            warehouse.isBlocked(
                position
            )
        );
    }
}


TEST(
    PathfinderTest,
    ReturnsEmptyPathWhenGoalIsUnreachable
) {
    Warehouse warehouse(3, 3);

    Position start{0, 0};
    Position goal{2, 2};

    warehouse.addObstacle({0, 1});
    warehouse.addObstacle({1, 0});

    std::vector<Position> path =
        findPath(
            warehouse,
            start,
            goal
        );

    EXPECT_TRUE(
        path.empty()
    );
}