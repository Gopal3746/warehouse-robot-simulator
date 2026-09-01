#include <gtest/gtest.h>

#include <vector>

#include "dispatcher.hpp"
#include "robot.hpp"
#include "task.hpp"
#include "warehouse.hpp"


TEST(
    DispatcherTest,
    SelectsRobotWithShortestPathToPickup
) {
    Warehouse warehouse(5, 5);

    std::vector<Robot> robots = {
        Robot(1, {0, 0}),
        Robot(2, {4, 4}),
        Robot(3, {2, 0})
    };

    Task task{
        1,
        {4, 3},
        {0, 0}
    };

    int selectedRobotIndex =
        findBestRobot(
            robots,
            warehouse,
            task
        );

    ASSERT_NE(
        selectedRobotIndex,
        -1
    );

    EXPECT_EQ(
        robots[selectedRobotIndex].getId(),
        2
    );
}


TEST(
    DispatcherTest,
    IgnoresBusyRobots
) {
    Warehouse warehouse(5, 5);

    std::vector<Robot> robots = {
        Robot(1, {0, 0}),
        Robot(2, {4, 4}),
        Robot(3, {2, 0})
    };

    robots[1].setState(
        RobotState::MovingToPickup
    );

    Task task{
        1,
        {4, 3},
        {0, 0}
    };

    int selectedRobotIndex =
        findBestRobot(
            robots,
            warehouse,
            task
        );

    ASSERT_NE(
        selectedRobotIndex,
        -1
    );

    EXPECT_EQ(
        robots[selectedRobotIndex].getId(),
        3
    );
}


TEST(
    DispatcherTest,
    ReturnsNegativeOneWhenPickupIsUnreachable
) {
    Warehouse warehouse(3, 3);

    std::vector<Robot> robots = {
        Robot(1, {0, 0}),
        Robot(2, {2, 2})
    };

    Position pickup{1, 1};

    warehouse.addObstacle({0, 1});
    warehouse.addObstacle({1, 0});
    warehouse.addObstacle({1, 2});
    warehouse.addObstacle({2, 1});

    Task task{
        1,
        pickup,
        {0, 0}
    };

    int selectedRobotIndex =
        findBestRobot(
            robots,
            warehouse,
            task
        );

    EXPECT_EQ(
        selectedRobotIndex,
        -1
    );
}