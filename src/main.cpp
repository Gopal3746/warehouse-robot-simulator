#include <cstddef>
#include <iostream>
#include <queue>
#include <string>
#include <vector>

#include "dispatcher.hpp"
#include "pathfinder.hpp"
#include "robot.hpp"
#include "task.hpp"
#include "warehouse.hpp"


struct RobotPlan {
    bool active = false;

    Task task{
        -1,
        {0, 0},
        {0, 0}
    };
};


bool samePosition(
    Position a,
    Position b
) {
    return a.row == b.row &&
           a.col == b.col;
}


bool hasActivePlans(
    const std::vector<RobotPlan>& plans
) {
    for (const RobotPlan& plan : plans) {
        if (plan.active) {
            return true;
        }
    }

    return false;
}


bool assignNextTask(
    std::queue<Task>& tasks,
    std::vector<Robot>& robots,
    std::vector<RobotPlan>& plans,
    const Warehouse& warehouse
) {
    if (tasks.empty()) {
        return false;
    }

    Task task = tasks.front();

    int selectedRobotIndex =
        findBestRobot(
            robots,
            warehouse,
            task
        );

    if (selectedRobotIndex == -1) {
        return false;
    }

    std::vector<Position> dropoffPath =
        findPath(
            warehouse,
            task.pickup,
            task.dropoff
        );

    if (dropoffPath.empty()) {
        std::cout
            << "Task "
            << task.id
            << " has no valid pickup-to-dropoff route. "
            << "Removing task.\n";

        tasks.pop();

        return true;
    }

    Robot& robot =
        robots[selectedRobotIndex];

    RobotPlan& plan =
        plans[selectedRobotIndex];

    plan.active = true;
    plan.task = task;

    robot.setState(
        RobotState::MovingToPickup
    );

    tasks.pop();

    std::cout
        << "Dispatcher assigned Task "
        << task.id
        << " to Robot "
        << robot.getId()
        << ".\n";

    return true;
}


void processArrival(
    Robot& robot,
    RobotPlan& plan
) {
    if (!plan.active) {
        return;
    }

    if (
        robot.getState() ==
            RobotState::MovingToPickup &&
        samePosition(
            robot.getPosition(),
            plan.task.pickup
        )
    ) {
        robot.setState(
            RobotState::Carrying
        );

        std::cout
            << "Robot "
            << robot.getId()
            << " picked up Task "
            << plan.task.id
            << ".\n";

        robot.setState(
            RobotState::MovingToDropoff
        );
    }

    if (
        robot.getState() ==
            RobotState::MovingToDropoff &&
        samePosition(
            robot.getPosition(),
            plan.task.dropoff
        )
    ) {
        std::cout
            << "Robot "
            << robot.getId()
            << " completed Task "
            << plan.task.id
            << ".\n";

        robot.setState(
            RobotState::Idle
        );

        plan.active = false;
    }
}


Position getTarget(
    const Robot& robot,
    const RobotPlan& plan
) {
    if (
        robot.getState() ==
        RobotState::MovingToPickup
    ) {
        return plan.task.pickup;
    }

    return plan.task.dropoff;
}


Warehouse buildDynamicWarehouse(
    const Warehouse& warehouse,
    const std::vector<Robot>& robots,
    std::size_t robotIndex
) {
    Warehouse dynamicWarehouse =
        warehouse;

    for (
        std::size_t i = 0;
        i < robots.size();
        i++
    ) {
        if (i == robotIndex) {
            continue;
        }

        dynamicWarehouse.addObstacle(
            robots[i].getPosition()
        );
    }

    return dynamicWarehouse;
}


Position proposeNextPosition(
    std::size_t robotIndex,
    const std::vector<Robot>& robots,
    const std::vector<RobotPlan>& plans,
    const Warehouse& warehouse
) {
    const Robot& robot =
        robots[robotIndex];

    const RobotPlan& plan =
        plans[robotIndex];

    Position currentPosition =
        robot.getPosition();

    if (!plan.active) {
        return currentPosition;
    }

    Position target =
        getTarget(
            robot,
            plan
        );

    Warehouse dynamicWarehouse =
        buildDynamicWarehouse(
            warehouse,
            robots,
            robotIndex
        );

    std::vector<Position> path =
        findPath(
            dynamicWarehouse,
            currentPosition,
            target
        );

    if (path.size() < 2) {
        return currentPosition;
    }

    return path[1];
}


void printFleetStatus(
    const std::vector<Robot>& robots,
    const std::vector<RobotPlan>& plans
) {
    std::cout << "\nFleet status:\n";

    for (
        std::size_t i = 0;
        i < robots.size();
        i++
    ) {
        const Robot& robot =
            robots[i];

        Position position =
            robot.getPosition();

        std::cout
            << "Robot "
            << robot.getId()
            << " | "
            << robotStateToString(
                robot.getState()
            )
            << " | Position ("
            << position.row
            << ", "
            << position.col
            << ")";

        if (plans[i].active) {
            std::cout
                << " | Task "
                << plans[i].task.id;
        }

        std::cout << '\n';
    }
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

    std::vector<RobotPlan> plans(
        robots.size()
    );

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

    std::cout
        << "Warehouse Robot Simulator\n"
        << "Collision-aware coordination enabled\n";

    int tick = 0;

    while (
        !tasks.empty() ||
        hasActivePlans(plans)
    ) {
        std::cout
            << "\n============================\n"
            << "Tick "
            << tick
            << "\n"
            << "============================\n";

        // Assign tasks to all available robots.
        while (
            assignNextTask(
                tasks,
                robots,
                plans,
                warehouse
            )
        ) {
        }

        // Handle cases where a robot is already
        // standing on its pickup/dropoff location.
        for (
            std::size_t i = 0;
            i < robots.size();
            i++
        ) {
            processArrival(
                robots[i],
                plans[i]
            );
        }

        // -----------------------------------
        // Phase 1: propose movements
        // -----------------------------------

        std::vector<Position> proposals;

        for (
            std::size_t i = 0;
            i < robots.size();
            i++
        ) {
            proposals.push_back(
                proposeNextPosition(
                    i,
                    robots,
                    plans,
                    warehouse
                )
            );
        }

        // -----------------------------------
        // Phase 2: approve movements
        // -----------------------------------

        std::vector<bool> approved(
            robots.size(),
            true
        );

        std::vector<std::string> waitReasons(
            robots.size()
        );

        // A robot that proposes its current
        // position is simply waiting.
        for (
            std::size_t i = 0;
            i < robots.size();
            i++
        ) {
            if (
                samePosition(
                    proposals[i],
                    robots[i].getPosition()
                )
            ) {
                approved[i] = false;

                if (plans[i].active) {
                    waitReasons[i] =
                        "no collision-free route this tick";
                }
            }
        }

        // Vertex conflict:
        //
        // Robot A -> X
        // Robot B -> X
        //
        // Only one robot may enter X.
        for (
            std::size_t i = 0;
            i < robots.size();
            i++
        ) {
            for (
                std::size_t j = i + 1;
                j < robots.size();
                j++
            ) {
                if (
                    samePosition(
                        proposals[i],
                        robots[i].getPosition()
                    ) ||
                    samePosition(
                        proposals[j],
                        robots[j].getPosition()
                    )
                ) {
                    continue;
                }

                if (
                    samePosition(
                        proposals[i],
                        proposals[j]
                    )
                ) {
                    std::size_t winner;
                    std::size_t loser;

                    if (
                        robots[i].getId() <
                        robots[j].getId()
                    ) {
                        winner = i;
                        loser = j;
                    }
                    else {
                        winner = j;
                        loser = i;
                    }

                    approved[loser] = false;

                    waitReasons[loser] =
                        "destination reserved by Robot " +
                        std::to_string(
                            robots[winner].getId()
                        );

                    std::cout
                        << "Collision avoided at ("
                        << proposals[i].row
                        << ", "
                        << proposals[i].col
                        << "): Robot "
                        << robots[winner].getId()
                        << " has priority over Robot "
                        << robots[loser].getId()
                        << ".\n";
                }
            }
        }

        // -----------------------------------
        // Phase 3: commit approved movement
        // -----------------------------------

        for (
            std::size_t i = 0;
            i < robots.size();
            i++
        ) {
            if (!plans[i].active) {
                continue;
            }

            Robot& robot =
                robots[i];

            if (!approved[i]) {
                std::cout
                    << "Robot "
                    << robot.getId()
                    << " waits at ("
                    << robot.getPosition().row
                    << ", "
                    << robot.getPosition().col
                    << ")";

                if (!waitReasons[i].empty()) {
                    std::cout
                        << " - "
                        << waitReasons[i];
                }

                std::cout << ".\n";

                continue;
            }

            robot.moveTo(
                proposals[i]
            );

            std::cout
                << "Robot "
                << robot.getId()
                << " | "
                << robotStateToString(
                    robot.getState()
                )
                << " | Task "
                << plans[i].task.id
                << " -> ("
                << robot.getPosition().row
                << ", "
                << robot.getPosition().col
                << ")\n";
        }

        // All movement for this tick has now
        // happened. Process arrivals together.
        for (
            std::size_t i = 0;
            i < robots.size();
            i++
        ) {
            processArrival(
                robots[i],
                plans[i]
            );
        }

        printFleetStatus(
            robots,
            plans
        );

        tick++;
    }

    std::cout
        << "\n============================\n"
        << "Simulation complete\n"
        << "============================\n";

    printFleetStatus(
        robots,
        plans
    );

    return 0;
}