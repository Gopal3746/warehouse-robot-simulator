#include "dispatcher.hpp"

#include <limits>

#include "pathfinder.hpp"

int findBestRobot(
    const std::vector<Robot>& robots,
    const Warehouse& warehouse,
    const Task& task
) {
    int bestRobotIndex = -1;
    int shortestPathLength = std::numeric_limits<int>::max();

    for (int i = 0; i < static_cast<int>(robots.size()); i++) {

        const Robot& robot = robots[i];

        if (robot.getState() != RobotState::Idle) {
            continue;
        }

        std::vector<Position> path =
            findPath(
                warehouse,
                robot.getPosition(),
                task.pickup
            );

        if (path.empty()) {
            continue;
        }

        int pathLength =
            static_cast<int>(path.size()) - 1;

        if (pathLength < shortestPathLength) {
            shortestPathLength = pathLength;
            bestRobotIndex = i;
        }
    }

    return bestRobotIndex;
}