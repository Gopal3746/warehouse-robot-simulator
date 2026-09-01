#include <cstddef>
#include <iostream>
#include <queue>
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

    std::vector<Position> pickupPath;
    std::vector<Position> dropoffPath;

    std::size_t nextStep = 1;
};


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

    Robot& robot = robots[selectedRobotIndex];
    RobotPlan& plan = plans[selectedRobotIndex];

    std::vector<Position> pickupPath =
        findPath(
            warehouse,
            robot.getPosition(),
            task.pickup
        );

    std::vector<Position> dropoffPath =
        findPath(
            warehouse,
            task.pickup,
            task.dropoff
        );

    if (pickupPath.empty() || dropoffPath.empty()) {
        std::cout
            << "Task "
            << task.id
            << " has no valid route. Removing task.\n";

        tasks.pop();

        return true;
    }

    plan.active = true;
    plan.task = task;
    plan.pickupPath = pickupPath;
    plan.dropoffPath = dropoffPath;
    plan.nextStep = 1;

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


void advanceRobot(
    Robot& robot,
    RobotPlan& plan
) {
    if (!plan.active) {
        return;
    }

    // -------------------------
    // Move toward pickup
    // -------------------------

    if (
        robot.getState() ==
        RobotState::MovingToPickup
    ) {
        if (
            plan.nextStep <
            plan.pickupPath.size()
        ) {
            robot.moveTo(
                plan.pickupPath[plan.nextStep]
            );

            plan.nextStep++;

            std::cout
                << "Robot "
                << robot.getId()
                << " moving to pickup for Task "
                << plan.task.id
                << " -> ("
                << robot.getPosition().row
                << ", "
                << robot.getPosition().col
                << ")\n";
        }

        if (
            plan.nextStep >=
            plan.pickupPath.size()
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

            plan.nextStep = 1;
        }

        return;
    }

    // -------------------------
    // Move toward dropoff
    // -------------------------

    if (
        robot.getState() ==
        RobotState::MovingToDropoff
    ) {
        if (
            plan.nextStep <
            plan.dropoffPath.size()
        ) {
            robot.moveTo(
                plan.dropoffPath[plan.nextStep]
            );

            plan.nextStep++;

            std::cout
                << "Robot "
                << robot.getId()
                << " moving to dropoff for Task "
                << plan.task.id
                << " -> ("
                << robot.getPosition().row
                << ", "
                << robot.getPosition().col
                << ")\n";
        }

        if (
            plan.nextStep >=
            plan.dropoffPath.size()
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
            plan.nextStep = 1;
        }
    }
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
        const Robot& robot = robots[i];

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
        << "Time-step coordination enabled\n";

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

        // Assign as many tasks as possible
        // to currently idle robots.
        while (
            assignNextTask(
                tasks,
                robots,
                plans,
                warehouse
            )
        ) {
        }

        // Every active robot advances
        // exactly one step this tick.
        for (
            std::size_t i = 0;
            i < robots.size();
            i++
        ) {
            advanceRobot(
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