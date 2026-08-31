#include "warehouse.hpp"

#include <iostream>

Warehouse::Warehouse(int rows, int cols)
    : rows_(rows),
      cols_(cols),
      grid_(rows, std::vector<char>(cols, '.')) {
}

bool Warehouse::isInside(Position position) const {
    return position.row >= 0 &&
           position.row < rows_ &&
           position.col >= 0 &&
           position.col < cols_;
}

bool Warehouse::isBlocked(Position position) const {
    if (!isInside(position)) {
        return true;
    }

    return grid_[position.row][position.col] == '#';
}

void Warehouse::addObstacle(Position position) {
    if (isInside(position)) {
        grid_[position.row][position.col] = '#';
    }
}

int Warehouse::getRows() const {
    return rows_;
}

int Warehouse::getCols() const {
    return cols_;
}

void Warehouse::print(
    Position robotPosition,
    Position goalPosition
) const {
    for (int row = 0; row < rows_; row++) {
        for (int col = 0; col < cols_; col++) {
            if (
                row == robotPosition.row &&
                col == robotPosition.col
            ) {
                std::cout << "R ";
            }
            else if (
                row == goalPosition.row &&
                col == goalPosition.col
            ) {
                std::cout << "G ";
            }
            else {
                std::cout << grid_[row][col] << ' ';
            }
        }

        std::cout << '\n';
    }
}
