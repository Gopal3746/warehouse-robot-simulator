#include <cstddef>
#include <iostream>
#include <vector>

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

int main() {
    Warehouse warehouse(5, 8);

    warehouse.addObstacle({1, 2});
    warehouse.addObstacle({1, 3});
    warehouse.addObstacle({2, 3});
    warehouse.addObstacle({3, 5});

    Robot robot(1, {0, 0});

    Task task{
        1,
        {2, 6},
        {4, 1}
    };

    std::cout << "Warehouse Robot Simulator\n";

    std::cout
        << "\nTask "
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
        << ")\n";

    std::cout << "\nInitial warehouse:\n\n";

    warehouse.print(
        robot.getPosition(),
        task.pickup
    );

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
        std::cout << "\nCould not reach pickup location.\n";
        return 1;
    }

    robot.setState(RobotState::Carrying);

    std::cout
        << "\nRobot "
        << robot.getId()
        << " picked up Task "
        << task.id
        << ".\n";

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
        std::cout << "\nCould not reach dropoff location.\n";
        return 1;
    }

    robot.setState(RobotState::Idle);

    std::cout
        << "\nTask "
        << task.id
        << " completed successfully.\n";

    std::cout
        << "Robot "
        << robot.getId()
        << " is now "
        << robotStateToString(robot.getState())
        << ".\n";

    return 0;
}
