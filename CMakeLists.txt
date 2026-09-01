cmake_minimum_required(VERSION 3.20)

project(warehouse_robot_simulator)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --------------------------------
# Core simulator library
# --------------------------------

add_library(
    warehouse_core
    src/warehouse.cpp
    src/robot.cpp
    src/pathfinder.cpp
    src/dispatcher.cpp
)

target_include_directories(
    warehouse_core
    PUBLIC
    include
)

# --------------------------------
# Main simulator executable
# --------------------------------

add_executable(
    warehouse_sim
    src/main.cpp
)

target_link_libraries(
    warehouse_sim
    PRIVATE
    warehouse_core
)

# --------------------------------
# Testing
# --------------------------------

include(CTest)

if(BUILD_TESTING)

    include(FetchContent)

    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.15.2
        GIT_SHALLOW TRUE
    )

    FetchContent_MakeAvailable(
        googletest
    )

    add_subdirectory(tests)

endif()