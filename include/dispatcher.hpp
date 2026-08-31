#pragma once

#include <vector>

#include "robot.hpp"
#include "task.hpp"
#include "warehouse.hpp"

int findBestRobot(
    const std::vector<Robot>& robots,
    const Warehouse& warehouse,
    const Task& task
);