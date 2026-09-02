add_executable(
    warehouse_tests
    robot_test.cpp
    pathfinder_test.cpp
    dispatcher_test.cpp
    simulation_test.cpp
)

target_link_libraries(
    warehouse_tests
    PRIVATE
    warehouse_core
    GTest::gtest_main
)

include(GoogleTest)

gtest_discover_tests(
    warehouse_tests
)