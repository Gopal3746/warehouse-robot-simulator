#include "robot.hpp"

Robot::Robot(int id, Position startPosition)
    : id_(id),
      position_(startPosition),
      state_(RobotState::Idle) {
}

int Robot::getId() const {
    return id_;
}

Position Robot::getPosition() const {
    return position_;
}

RobotState Robot::getState() const {
    return state_;
}

void Robot::moveTo(Position newPosition) {
    position_ = newPosition;
}

void Robot::setState(RobotState newState) {
    state_ = newState;
}

std::string robotStateToString(RobotState state) {
    switch (state) {
        case RobotState::Idle:
            return "Idle";

        case RobotState::MovingToPickup:
            return "Moving to pickup";

        case RobotState::Carrying:
            return "Carrying item";

        case RobotState::MovingToDropoff:
            return "Moving to dropoff";
    }

    return "Unknown";
}
