#include "simulation.hpp"

#include <iostream>
#include <string>
#include <vector>

#include "dispatcher.hpp"
#include "pathfinder.hpp"


Simulation::Simulation(
    const Warehouse& warehouse,
    const std::vector<Robot>& robots
)
    : warehouse_(warehouse),
      robots_(robots),
      plans_(robots.size()) {
}


void Simulation::addTask(Task task) {
    tasks_.push(task);
}


bool Simulation::samePosition(
    Position a,
    Position b
) {
    return a.row == b.row &&
           a.col == b.col;
}


bool Simulation::hasActivePlans() const {
    for (const RobotPlan& plan : plans_) {
        if (plan.active) {
            return true;
        }
    }

    return false;
}


bool Simulation::isComplete() const {
    return tasks_.empty() &&
           !hasActivePlans();
}


int Simulation::getTick() const {
    return tick_;
}


int Simulation::getCompletedTaskCount() const {
    return completedTaskCount_;
}


const std::vector<Robot>&
Simulation::getRobots() const {
    return robots_;
}


bool Simulation::hasPositionCollision() const {
    for (
        std::size_t i = 0;
        i < robots_.size();
        i++
    ) {
        for (
            std::size_t j = i + 1;
            j < robots_.size();
            j++
        ) {
            if (
                samePosition(
                    robots_[i].getPosition(),
                    robots_[j].getPosition()
                )
            ) {
                return true;
            }
        }
    }

    return false;
}


bool Simulation::assignNextTask() {
    if (tasks_.empty()) {
        return false;
    }

    Task task = tasks_.front();

    int selectedRobotIndex =
        findBestRobot(
            robots_,
            warehouse_,
            task
        );

    if (selectedRobotIndex == -1) {
        return false;
    }

    std::vector<Position> dropoffPath =
        findPath(
            warehouse_,
            task.pickup,
            task.dropoff
        );

    if (dropoffPath.empty()) {
        std::cout
            << "Task "
            << task.id
            << " has no valid pickup-to-dropoff route. "
            << "Removing task.\n";

        tasks_.pop();

        return true;
    }

    Robot& robot =
        robots_[selectedRobotIndex];

    RobotPlan& plan =
        plans_[selectedRobotIndex];

    plan.active = true;
    plan.task = task;

    robot.setState(
        RobotState::MovingToPickup
    );

    tasks_.pop();

    std::cout
        << "Dispatcher assigned Task "
        << task.id
        << " to Robot "
        << robot.getId()
        << ".\n";

    return true;
}


void Simulation::processArrival(
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

        completedTaskCount_++;
    }
}


Position Simulation::getTarget(
    const Robot& robot,
    const RobotPlan& plan
) const {
    if (
        robot.getState() ==
        RobotState::MovingToPickup
    ) {
        return plan.task.pickup;
    }

    return plan.task.dropoff;
}


Warehouse Simulation::buildDynamicWarehouse(
    std::size_t robotIndex
) const {
    Warehouse dynamicWarehouse =
        warehouse_;

    for (
        std::size_t i = 0;
        i < robots_.size();
        i++
    ) {
        if (i == robotIndex) {
            continue;
        }

        dynamicWarehouse.addObstacle(
            robots_[i].getPosition()
        );
    }

    return dynamicWarehouse;
}


Position Simulation::proposeNextPosition(
    std::size_t robotIndex
) const {
    const Robot& robot =
        robots_[robotIndex];

    const RobotPlan& plan =
        plans_[robotIndex];

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


void Simulation::printFleetStatus() const {
    std::cout << "\nFleet status:\n";

    for (
        std::size_t i = 0;
        i < robots_.size();
        i++
    ) {
        const Robot& robot =
            robots_[i];

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

        if (plans_[i].active) {
            std::cout
                << " | Task "
                << plans_[i].task.id;
        }

        std::cout << '\n';
    }
}


void Simulation::tick() {
    std::cout
        << "\n============================\n"
        << "Tick "
        << tick_
        << "\n"
        << "============================\n";

    // Assign as many tasks as possible
    // to available robots.
    while (assignNextTask()) {
    }

    // Handle robots already standing
    // on pickup/dropoff cells.
    for (
        std::size_t i = 0;
        i < robots_.size();
        i++
    ) {
        processArrival(
            robots_[i],
            plans_[i]
        );
    }

    // --------------------------------
    // Phase 1: propose movements
    // --------------------------------

    std::vector<Position> proposals;

    for (
        std::size_t i = 0;
        i < robots_.size();
        i++
    ) {
        proposals.push_back(
            proposeNextPosition(i)
        );
    }

    // --------------------------------
    // Phase 2: resolve conflicts
    // --------------------------------

    std::vector<bool> approved(
        robots_.size(),
        true
    );

    std::vector<std::string> waitReasons(
        robots_.size()
    );

    for (
        std::size_t i = 0;
        i < robots_.size();
        i++
    ) {
        if (
            samePosition(
                proposals[i],
                robots_[i].getPosition()
            )
        ) {
            approved[i] = false;

            if (plans_[i].active) {
                waitReasons[i] =
                    "no collision-free route this tick";
            }
        }
    }

    // Vertex conflicts:
    //
    // R1 -> X
    // R2 -> X
    //
    // Lower robot ID currently wins.

    for (
        std::size_t i = 0;
        i < robots_.size();
        i++
    ) {
        for (
            std::size_t j = i + 1;
            j < robots_.size();
            j++
        ) {
            if (
                samePosition(
                    proposals[i],
                    robots_[i].getPosition()
                ) ||
                samePosition(
                    proposals[j],
                    robots_[j].getPosition()
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
                    robots_[i].getId() <
                    robots_[j].getId()
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
                        robots_[winner].getId()
                    );

                std::cout
                    << "Collision avoided at ("
                    << proposals[i].row
                    << ", "
                    << proposals[i].col
                    << "): Robot "
                    << robots_[winner].getId()
                    << " has priority over Robot "
                    << robots_[loser].getId()
                    << ".\n";
            }
        }
    }

    // --------------------------------
    // Phase 3: commit movements
    // --------------------------------

    for (
        std::size_t i = 0;
        i < robots_.size();
        i++
    ) {
        if (!plans_[i].active) {
            continue;
        }

        Robot& robot =
            robots_[i];

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
            << plans_[i].task.id
            << " -> ("
            << robot.getPosition().row
            << ", "
            << robot.getPosition().col
            << ")\n";
    }

    // Process pickup/dropoff after
    // all robots have moved.
    for (
        std::size_t i = 0;
        i < robots_.size();
        i++
    ) {
        processArrival(
            robots_[i],
            plans_[i]
        );
    }

    printFleetStatus();

    tick_++;
}


void Simulation::run() {
    while (!isComplete()) {
        tick();
    }

    std::cout
        << "\n============================\n"
        << "Simulation complete\n"
        << "============================\n";

    printFleetStatus();
}