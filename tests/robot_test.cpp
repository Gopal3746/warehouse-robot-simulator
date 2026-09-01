#include <gtest/gtest.h>

#include "robot.hpp"


TEST(RobotTest, StartsIdleAtGivenPosition) {
    Robot robot(1, {2, 3});

    EXPECT_EQ(robot.getId(), 1);
    EXPECT_EQ(robot.getPosition().row, 2);
    EXPECT_EQ(robot.getPosition().col, 3);

    EXPECT_EQ(
        robot.getState(),
        RobotState::Idle
    );
}


TEST(RobotTest, MoveUpdatesPosition) {
    Robot robot(1, {0, 0});

    robot.moveTo({2, 4});

    EXPECT_EQ(
        robot.getPosition().row,
        2
    );

    EXPECT_EQ(
        robot.getPosition().col,
        4
    );
}


TEST(RobotTest, StateCanChange) {
    Robot robot(1, {0, 0});

    robot.setState(
        RobotState::MovingToPickup
    );

    EXPECT_EQ(
        robot.getState(),
        RobotState::MovingToPickup
    );

    robot.setState(
        RobotState::MovingToDropoff
    );

    EXPECT_EQ(
        robot.getState(),
        RobotState::MovingToDropoff
    );

    robot.setState(
        RobotState::Idle
    );

    EXPECT_EQ(
        robot.getState(),
        RobotState::Idle
    );
}