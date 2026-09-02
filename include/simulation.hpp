#pragma once

#include <cstddef>
#include <queue>
#include <vector>

#include "robot.hpp"
#include "task.hpp"
#include "warehouse.hpp"


class Simulation {
public:
    Simulation(
        const Warehouse& warehouse,
        const std::vector<Robot>& robots
    );

    void addTask(Task task);

    void tick();
    void run();

    bool failRobot(int robotId);

    bool isComplete() const;
    bool hasPositionCollision() const;

    int getTick() const;
    int getCompletedTaskCount() const;

    const std::vector<Robot>& getRobots() const;

private:
    struct RobotPlan {
        bool active = false;

        Task task{
            -1,
            {0, 0},
            {0, 0}
        };
    };

    Warehouse warehouse_;
    std::vector<Robot> robots_;
    std::vector<RobotPlan> plans_;
    std::queue<Task> tasks_;

    int tick_ = 0;
    int completedTaskCount_ = 0;

    bool hasActivePlans() const;

    bool assignNextTask();

    void processArrival(
        Robot& robot,
        RobotPlan& plan
    );

    Position getTarget(
        const Robot& robot,
        const RobotPlan& plan
    ) const;

    Warehouse buildDynamicWarehouse(
        std::size_t robotIndex
    ) const;

    Position proposeNextPosition(
        std::size_t robotIndex
    ) const;

    void printFleetStatus() const;

    static bool samePosition(
        Position a,
        Position b
    );
};