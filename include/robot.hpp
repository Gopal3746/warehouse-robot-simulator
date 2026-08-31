#pragma once

#include <string>

#include "position.hpp"

enum class RobotState {
    Idle,
    MovingToPickup,
    Carrying,
    MovingToDropoff
};

class Robot {
public:
    Robot(int id, Position startPosition);

    int getId() const;
    Position getPosition() const;
    RobotState getState() const;

    void moveTo(Position newPosition);
    void setState(RobotState newState);

private:
    int id_;
    Position position_;
    RobotState state_;
};

std::string robotStateToString(RobotState state);
