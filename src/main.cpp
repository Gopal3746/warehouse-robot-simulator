#include <cstddef>
#include <iostream>
#include <queue>
#include <vector>

#include "dispatcher.hpp"
#include "pathfinder.hpp"
#include "robot.hpp"
#include "task.hpp"
#include "warehouse.hpp"

bool executePath(
    Robot& robot,
    const Warehouse& warehouse,
    const std::vector<Position>& path,
    Position target
) {
    if (path.empty()) {
        return false;
    }

    for (std::size_t i = 1; i < path.size(); i++) {
        robot.moveTo(path[i]);

        std::cout
            << "\nRobot "
            << robot.getId()
            << " | "
            << robotStateToString(robot.getState())
            << " | Position ("
            << robot.getPosition().row
            << ", "
            << robot.getPosition().col
            << ")\n\n";

        warehouse.print(
            robot.getPosition(),
            target
        );
    }

    return true;
}

bool executeTask(
    Robot& robot,
    const Warehouse& warehouse,
    const Task& task
) {
    // -------------------------
    // Travel to pickup
    // -------------------------

    robot.setState(RobotState::MovingToPickup);

    std::cout << "\nFinding path to pickup...\n";

    std::vector<Position> pickupPath =
        findPath(
            warehouse,
            robot.getPosition(),
            task.pickup
        );

    if (!executePath(
        robot,
        warehouse,
        pickupPath,
        task.pickup
    )) {
        std::cout
            << "\nRobot "
            << robot.getId()
            << " could not reach the pickup for Task "
            << task.id
            << ".\n";

        robot.setState(RobotState::Idle);

        return false;
    }

    // -------------------------
    // Pick up item
    // -------------------------

    robot.setState(RobotState::Carrying);

    std::cout
        << "\nRobot "
        << robot.getId()
        << " picked up Task "
        << task.id
        << ".\n";

    // -------------------------
    // Travel to dropoff
    // -------------------------

    robot.setState(RobotState::MovingToDropoff);

    std::cout << "\nFinding path to dropoff...\n";

    std::vector<Position> dropoffPath =
        findPath(
            warehouse,
            robot.getPosition(),
            task.dropoff
        );

    if (!executePath(
        robot,
        warehouse,
        dropoffPath,
        task.dropoff
    )) {
        std::cout
            << "\nRobot "
            << robot.getId()
            << " could not reach the dropoff for Task "
            << task.id
            << ".\n";

        robot.setState(RobotState::Idle);

        return false;
    }

    // -------------------------
    // Task complete
    // -------------------------

    robot.setState(RobotState::Idle);

    std::cout
        << "\nTask "
        << task.id
        << " completed successfully by Robot "
        << robot.getId()
        << ".\n";

    return true;
}

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

    std::queue<Task> tasks;

    tasks.push(Task{
        1,
        {2, 6},
        {4, 1}
    });

    tasks.push(Task{
        2,
        {0, 7},
        {3, 4}
    });

    tasks.push(Task{
        3,
        {4, 6},
        {0, 1}
    });

    tasks.push(Task{
        4,
        {2, 0},
        {4, 7}
    });

    std::cout << "Warehouse Robot Simulator\n";

    while (!tasks.empty()) {
        Task task = tasks.front();
        tasks.pop();

        std::cout
            << "\n============================\n"
            << "Task "
            << task.id
            << "\nPickup: ("
            << task.pickup.row
            << ", "
            << task.pickup.col
            << ")"
            << "\nDropoff: ("
            << task.dropoff.row
            << ", "
            << task.dropoff.col
            << ")"
            << "\n============================\n";

        int selectedRobotIndex =
            findBestRobot(
                robots,
                warehouse,
                task
            );

        if (selectedRobotIndex == -1) {
            std::cout
                << "\nNo available robot can reach Task "
                << task.id
                << ". Skipping task.\n";

            continue;
        }

        Robot& robot = robots[selectedRobotIndex];

        std::cout
            << "\nDispatcher selected Robot "
            << robot.getId()
            << ".\n";

        std::cout << "\nCurrent warehouse:\n\n";

        warehouse.print(
            robot.getPosition(),
            task.pickup
        );

        executeTask(
            robot,
            warehouse,
            task
        );
    }

    std::cout
        << "\n============================\n"
        << "Simulation complete\n"
        << "============================\n";

    for (const Robot& robot : robots) {
        Position position = robot.getPosition();

        std::cout
            << "Robot "
            << robot.getId()
            << " | "
            << robotStateToString(robot.getState())
            << " | Final position ("
            << position.row
            << ", "
            << position.col
            << ")\n";
    }

    return 0;
}