cmake_minimum_required(VERSION 3.20)

project(warehouse_robot_simulator)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(
    warehouse_sim
    src/main.cpp
    src/warehouse.cpp
    src/robot.cpp
    src/pathfinder.cpp
    src/dispatcher.cpp
)

target_include_directories(
    warehouse_sim
    PRIVATE
    include
)
