#pragma once

#include <vector>

#include "position.hpp"

class Warehouse {
public:
    Warehouse(int rows, int cols);

    void addObstacle(Position position);

    bool isInside(Position position) const;
    bool isBlocked(Position position) const;

    int getRows() const;
    int getCols() const;

    void print(Position robotPosition, Position goalPosition) const;

private:
    int rows_;
    int cols_;

    std::vector<std::vector<char>> grid_;
};
