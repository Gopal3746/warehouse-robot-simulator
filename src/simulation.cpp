#include "simulation.hpp"

#include <iomanip>
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

    for (const Robot& robot : robots_) {
        metrics_.robots.push_back({
            robot.getId(),
            0,
            0,
            0
        });
    }
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
    return metrics_.completedTasks;
}


const SimulationMetrics&
Simulation::getMetrics() const {
    return metrics_;
}


const std::vector<Robot>&
Simulation::getRobots() const {
    return robots_;
}


bool Simulation::failRobot(
    int robotId
) {
    for (
        std::size_t i = 0;
        i < robots_.size();
        i++
    ) {
        Robot& robot =
            robots_[i];

        if (robot.getId() != robotId) {
            continue;
        }

        if (
            robot.getState() ==
            RobotState::Failed
        ) {
            return false;
        }

        metrics_.failures++;

        Position failurePosition =
            robot.getPosition();

        RobotPlan& plan =
            plans_[i];

        if (plan.active) {
            Task recoveryTask =
                plan.task;

            bool carryingItem =
                robot.getState() ==
                    RobotState::Carrying ||
                robot.getState() ==
                    RobotState::MovingToDropoff;

            if (carryingItem) {
                recoveryTask.pickup =
                    failurePosition;

                std::cout
                    << "Robot "
                    << robot.getId()
                    << " failed while carrying Task "
                    << recoveryTask.id
                    << ".\n";

                std::cout
                    << "Recovery pickup moved to ("
                    << failurePosition.row
                    << ", "
                    << failurePosition.col
                    << ").\n";
            }
            else {
                std::cout
                    << "Robot "
                    << robot.getId()
                    << " failed while traveling to Task "
                    << recoveryTask.id
                    << " pickup.\n";
            }

            tasks_.push(
                recoveryTask
            );

            metrics_.recoveredTasks++;

            plan.active = false;

            std::cout
                << "Task "
                << recoveryTask.id
                << " returned to the task queue.\n";
        }

        robot.setState(
            RobotState::Failed
        );

        // Failed robots are removed
        // from the active warehouse floor.
        robot.moveTo({
            -1,
            -1
        });

        std::cout
            << "Robot "
            << robot.getId()
            << " is now unavailable.\n";

        return true;
    }

    return false;
}


bool Simulation::hasPositionCollision() const {
    for (
        std::size_t i = 0;
        i < robots_.size();
        i++
    ) {
        if (
            robots_[i].getState() ==
            RobotState::Failed
        ) {
            continue;
        }

        for (
            std::size_t j = i + 1;
            j < robots_.size();
            j++
        ) {
            if (
                robots_[j].getState() ==
                RobotState::Failed
            ) {
                continue;
            }

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

    Task task =
        tasks_.front();

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

        metrics_.completedTasks++;
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
        if (
            i == robotIndex ||
            robots_[i].getState() ==
                RobotState::Failed
        ) {
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
    std::cout
        << "\nFleet status:\n";

    for (
        std::size_t i = 0;
        i < robots_.size();
        i++
    ) {
        const Robot& robot =
            robots_[i];

        if (
            robot.getState() ==
            RobotState::Failed
        ) {
            std::cout
                << "Robot "
                << robot.getId()
                << " | Failed"
                << " | Removed from active floor\n";

            continue;
        }

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


void Simulation::printMetrics() const {
    std::cout
        << "\n============================\n"
        << "Simulation Metrics\n"
        << "============================\n";

    std::cout
        << "Total ticks: "
        << metrics_.totalTicks
        << '\n';

    std::cout
        << "Tasks completed: "
        << metrics_.completedTasks
        << '\n';

    std::cout
        << "Robot movements: "
        << metrics_.totalMovements
        << '\n';

    std::cout
        << "Wait events: "
        << metrics_.totalWaits
        << '\n';

    std::cout
        << "Robot failures: "
        << metrics_.failures
        << '\n';

    std::cout
        << "Recovered tasks: "
        << metrics_.recoveredTasks
        << '\n';

    std::cout
        << "\nPer-robot metrics:\n";

    for (
        const RobotMetrics& robotMetrics :
        metrics_.robots
    ) {
        double utilization = 0.0;

        if (metrics_.totalTicks > 0) {
            utilization =
                100.0 *
                static_cast<double>(
                    robotMetrics.activeTicks
                ) /
                static_cast<double>(
                    metrics_.totalTicks
                );
        }

        std::cout
            << "Robot "
            << robotMetrics.robotId
            << " | Active ticks: "
            << robotMetrics.activeTicks
            << " | Moves: "
            << robotMetrics.movementCount
            << " | Waits: "
            << robotMetrics.waitCount
            << " | Utilization: "
            << std::fixed
            << std::setprecision(1)
            << utilization
            << "%\n";
    }
}


void Simulation::tick() {
    std::cout
        << "\n============================\n"
        << "Tick "
        << tick_
        << "\n"
        << "============================\n";

    // --------------------------------
    // Assign available tasks
    // --------------------------------

    while (assignNextTask()) {
    }

    // --------------------------------
    // Process robots already at target
    // --------------------------------

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
    // Record utilization
    // --------------------------------

    for (
        std::size_t i = 0;
        i < plans_.size();
        i++
    ) {
        if (plans_[i].active) {
            metrics_.robots[i].activeTicks++;
        }
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

    // Robots that cannot find a movement
    // remain in their current position.
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

    // Vertex conflict:
    //
    // R1 -> X
    // R2 -> X
    //
    // Lower robot ID gets priority.
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

                approved[loser] =
                    false;

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
            metrics_.totalWaits++;
            metrics_.robots[i].waitCount++;

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

        metrics_.totalMovements++;
        metrics_.robots[i].movementCount++;

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

    // --------------------------------
    // Process arrivals after movement
    // --------------------------------

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
    metrics_.totalTicks++;
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

    printMetrics();
}