#include "pathfinder.hpp"

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <queue>
#include <vector>

namespace {

struct Node {
    Position position;
    int gCost;
    int fCost;
};

struct CompareNode {
    bool operator()(const Node& a, const Node& b) const {
        return a.fCost > b.fCost;
    }
};

int heuristic(Position a, Position b) {
    return std::abs(a.row - b.row) +
           std::abs(a.col - b.col);
}

bool samePosition(Position a, Position b) {
    return a.row == b.row &&
           a.col == b.col;
}

}

std::vector<Position> findPath(
    const Warehouse& warehouse,
    Position start,
    Position goal
) {
    if (
        !warehouse.isInside(start) ||
        !warehouse.isInside(goal) ||
        warehouse.isBlocked(start) ||
        warehouse.isBlocked(goal)
    ) {
        return {};
    }

    int rows = warehouse.getRows();
    int cols = warehouse.getCols();

    const int infinity = std::numeric_limits<int>::max();

    std::vector<std::vector<int>> gScore(
        rows,
        std::vector<int>(cols, infinity)
    );

    std::vector<std::vector<Position>> parent(
        rows,
        std::vector<Position>(cols, {-1, -1})
    );

    std::vector<std::vector<bool>> visited(
        rows,
        std::vector<bool>(cols, false)
    );

    std::priority_queue<
        Node,
        std::vector<Node>,
        CompareNode
    > openNodes;

    gScore[start.row][start.col] = 0;

    openNodes.push({
        start,
        0,
        heuristic(start, goal)
    });

    std::vector<Position> directions = {
        {-1, 0},
        {1, 0},
        {0, -1},
        {0, 1}
    };

    while (!openNodes.empty()) {
        Node current = openNodes.top();
        openNodes.pop();

        Position currentPosition = current.position;

        if (visited[currentPosition.row][currentPosition.col]) {
            continue;
        }

        visited[currentPosition.row][currentPosition.col] = true;

        if (samePosition(currentPosition, goal)) {
            std::vector<Position> path;
            Position step = goal;

            while (!samePosition(step, start)) {
                path.push_back(step);
                step = parent[step.row][step.col];
            }

            path.push_back(start);

            std::reverse(path.begin(), path.end());

            return path;
        }

        for (Position direction : directions) {
            Position nextPosition{
                currentPosition.row + direction.row,
                currentPosition.col + direction.col
            };

            if (
                !warehouse.isInside(nextPosition) ||
                warehouse.isBlocked(nextPosition)
            ) {
                continue;
            }

            if (visited[nextPosition.row][nextPosition.col]) {
                continue;
            }

            int newGCost =
                gScore[currentPosition.row][currentPosition.col] + 1;

            if (
                newGCost <
                gScore[nextPosition.row][nextPosition.col]
            ) {
                gScore[nextPosition.row][nextPosition.col] = newGCost;

                parent[nextPosition.row][nextPosition.col] =
                    currentPosition;

                int newFCost =
                    newGCost +
                    heuristic(nextPosition, goal);

                openNodes.push({
                    nextPosition,
                    newGCost,
                    newFCost
                });
            }
        }
    }

    return {};
}
