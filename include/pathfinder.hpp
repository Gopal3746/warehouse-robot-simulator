#pragma once

#include <vector>

#include "position.hpp"
#include "warehouse.hpp"

std::vector<Position> findPath(
    const Warehouse& warehouse,
    Position start,
    Position goal
);
