#include <iostream>
#include <vector>

#include "robot.hpp"
#include "simulation.hpp"
#include "task.hpp"
#include "warehouse.hpp"


int main() {
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

    simulation.addTask(Task{
        1,
        {2, 6},
        {4, 1}
    });

    simulation.addTask(Task{
        2,
        {0, 7},
        {3, 4}
    });

    simulation.addTask(Task{
        3,
        {4, 6},
        {0, 1}
    });

    simulation.addTask(Task{
        4,
        {2, 0},
        {4, 7}
    });

    std::cout
        << "Warehouse Robot Simulator\n"
        << "Collision-aware coordination enabled\n";

    simulation.run();

    return 0;
}