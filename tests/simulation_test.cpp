#include <gtest/gtest.h>

#include <vector>

#include "robot.hpp"
#include "simulation.hpp"
#include "task.hpp"
#include "warehouse.hpp"


TEST(
    SimulationTest,
    CompletesQueuedTasks
) {
    Warehouse warehouse(5, 8);

    warehouse.addObstacle({1, 2});
    warehouse.addObstacle({1, 3});
    warehouse.addObstacle({2, 3});
    warehouse.addObstacle({3, 5});

    std::vector<Robot> robots = {
        Robot(1, {0, 0}),
        Robot(2, {4, 7}),
        Robot(3, {3, 0})
    };

    Simulation simulation(
        warehouse,
        robots
    );

    simulation.addTask({
        1,
        {2, 6},
        {4, 1}
    });

    simulation.addTask({
        2,
        {0, 7},
        {3, 4}
    });

    int safetyLimit = 100;

    while (
        !simulation.isComplete() &&
        safetyLimit > 0
    ) {
        simulation.tick();
        safetyLimit--;
    }

    EXPECT_TRUE(
        simulation.isComplete()
    );

    EXPECT_EQ(
        simulation.getCompletedTaskCount(),
        2
    );
}


TEST(
    SimulationTest,
    PreventsRobotPositionCollisions
) {
    Warehouse warehouse(5, 8);

    warehouse.addObstacle({1, 2});
    warehouse.addObstacle({1, 3});
    warehouse.addObstacle({2, 3});
    warehouse.addObstacle({3, 5});

    std::vector<Robot> robots = {
        Robot(1, {0, 0}),
        Robot(2, {4, 7}),
        Robot(3, {3, 0})
    };

    Simulation simulation(
        warehouse,
        robots
    );

    simulation.addTask({
        1,
        {2, 6},
        {4, 1}
    });

    simulation.addTask({
        2,
        {0, 7},
        {3, 4}
    });

    simulation.addTask({
        3,
        {4, 6},
        {0, 1}
    });

    int safetyLimit = 100;

    while (
        !simulation.isComplete() &&
        safetyLimit > 0
    ) {
        simulation.tick();

        EXPECT_FALSE(
            simulation.hasPositionCollision()
        );

        safetyLimit--;
    }

    EXPECT_TRUE(
        simulation.isComplete()
    );
}