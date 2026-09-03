# Warehouse Robot Simulator

A C++20 warehouse robot fleet simulator featuring A* pathfinding, multi-robot task dispatching, time-step coordination, collision-aware movement, dynamic replanning, failure recovery, simulation metrics, automated testing, and CI.

The simulator models robots navigating a grid-based warehouse while completing pickup-and-dropoff tasks. Multiple robots can operate during the same simulation tick, and the coordinator prevents robots from occupying the same warehouse cell.

## Features

* Grid-based warehouse representation with static obstacles
* A* shortest-path planning using Manhattan distance
* Multi-robot task dispatching
* FIFO task queue
* Time-step based robot movement
* Dynamic path replanning around other robots
* Collision-aware movement coordination
* Robot state machine
* Robot failure injection and task recovery
* Per-robot and fleet-wide simulation metrics
* GoogleTest unit and integration tests
* CMake build system
* GitHub Actions CI

## Robot States

Each robot moves through a small state machine:

```text
Idle
  |
  v
Moving to pickup
  |
  v
Carrying
  |
  v
Moving to dropoff
  |
  v
Idle
```

A robot can also transition to:

```text
Failed
```

Failed robots are removed from the active warehouse floor and are no longer considered for task dispatching or movement planning.

## Architecture

```text
                     +------------------+
                     |    Task Queue    |
                     +--------+---------+
                              |
                              v
                     +------------------+
                     |    Dispatcher    |
                     +--------+---------+
                              |
                    Select best idle robot
                              |
                              v
                     +------------------+
                     |    Simulation    |
                     |   Coordinator    |
                     +--------+---------+
                              |
          +-------------------+-------------------+
          |                   |                   |
          v                   v                   v
      +---------+         +---------+         +---------+
      | Robot 1 |         | Robot 2 |         | Robot 3 |
      +---------+         +---------+         +---------+
          |                   |                   |
          +-------------------+-------------------+
                              |
                              v
                     +------------------+
                     |   A* Pathfinder  |
                     +--------+---------+
                              |
                              v
                     +------------------+
                     |    Warehouse     |
                     | Grid + Obstacles |
                     +------------------+
```

The main components are:

* `Warehouse` — stores the grid and static obstacles.
* `Robot` — stores robot identity, position, and state.
* `Task` — represents a pickup and dropoff request.
* `Pathfinder` — computes shortest routes using A*.
* `Dispatcher` — assigns tasks to reachable idle robots.
* `Simulation` — coordinates task assignment, robot movement, collision handling, failures, recovery, and metrics.

## Project Structure

```text
warehouse-robot-simulator/
├── .github/
│   └── workflows/
│       └── ci.yml
├── include/
│   ├── dispatcher.hpp
│   ├── pathfinder.hpp
│   ├── position.hpp
│   ├── robot.hpp
│   ├── simulation.hpp
│   ├── task.hpp
│   └── warehouse.hpp
├── src/
│   ├── dispatcher.cpp
│   ├── main.cpp
│   ├── pathfinder.cpp
│   ├── robot.cpp
│   ├── simulation.cpp
│   └── warehouse.cpp
├── tests/
│   ├── CMakeLists.txt
│   ├── dispatcher_test.cpp
│   ├── pathfinder_test.cpp
│   ├── robot_test.cpp
│   └── simulation_test.cpp
├── .gitignore
├── CMakeLists.txt
└── README.md
```

## A* Pathfinding

Robots navigate the warehouse using A* search.

For each candidate cell:

```text
f(n) = g(n) + h(n)
```

where:

* `g(n)` is the movement cost from the starting position.
* `h(n)` is the estimated distance to the goal.
* `f(n)` is the estimated total path cost.

Because robots move only horizontally and vertically, the heuristic uses Manhattan distance:

```text
h =
|current_row - goal_row|
+
|current_col - goal_col|
```

Static obstacles are excluded from valid paths.

## Task Dispatching

When a task reaches the front of the queue, the dispatcher evaluates idle robots.

For every candidate robot:

1. Find an A* path from the robot to the task pickup.
2. Ignore robots that cannot reach the pickup.
3. Ignore robots that are busy or failed.
4. Compare path lengths.
5. Assign the task to the robot with the shortest reachable path.

This uses actual warehouse paths rather than straight-line distance, so obstacles influence robot selection.

## Time-Step Simulation

Robot movement is coordinated using discrete simulation ticks.

Instead of allowing one robot to complete an entire task before another robot moves:

```text
Robot 1: A -> B -> C -> D
Robot 2: E -> F -> G -> H
```

the simulator advances the fleet together:

```text
Tick 0
Robot 1 moves one cell
Robot 2 moves one cell
Robot 3 moves one cell

Tick 1
Robot 1 moves one cell
Robot 2 moves one cell
Robot 3 moves one cell
```

This provides deterministic fleet coordination and makes movement conflicts observable and testable.

## Collision-Aware Movement

Each active robot calculates a proposed next position before movement is committed.

The simulation uses three phases:

```text
1. Propose movement
        |
        v
2. Resolve conflicts
        |
        v
3. Commit approved movement
```

### Occupied Cells

Before running A*, the simulator creates a temporary warehouse representation for each robot.

The positions of other active robots are temporarily treated as obstacles.

This allows robots to dynamically plan around one another.

### Vertex Conflicts

Two robots may independently propose the same currently empty cell:

```text
Robot 1 ----\
             > (x, y)
Robot 2 ----/
```

Only one movement is approved.

The current deterministic priority rule gives the lower robot ID priority while the other robot waits for the tick.

## Dynamic Replanning

Paths are recalculated as the fleet moves.

A robot therefore does not blindly follow a route calculated at the beginning of its task.

Instead:

```text
current position
      |
      v
observe current warehouse state
      |
      v
treat other robots as obstacles
      |
      v
run A*
      |
      v
propose next movement
```

This allows robots to adapt when another robot blocks a previously available route.

## Failure Recovery

The simulator supports explicit robot failure injection.

If a robot fails while traveling toward a pickup:

```text
Robot fails
    |
    v
Task returns to queue
    |
    v
Another robot may receive it
```

If a robot fails after collecting the item, the recovery location becomes the failed robot's last valid warehouse position:

```text
Robot 2 carrying Task 1
          |
          v
      Robot fails
          |
          v
Recovery pickup = failure position
          |
          v
Task returns to queue
          |
          v
Robot 3 retrieves item
          |
          v
Task continues to dropoff
```

Failed robots transition to the `Failed` state and are removed from the active warehouse floor.

## Simulation Metrics

The simulator records both fleet-wide and per-robot statistics.

Fleet metrics include:

* total simulation ticks
* completed tasks
* robot movements
* wait events
* robot failures
* recovered tasks

Per-robot metrics include:

* active ticks
* successful movements
* waits
* utilization

Utilization is calculated as:

```text
utilization =
active ticks / total simulation ticks
```

### Example Run

The default failure-injection scenario produces:

```text
Simulation Metrics
============================
Total ticks: 31
Tasks completed: 4
Robot movements: 62
Wait events: 1
Robot failures: 1
Recovered tasks: 1

Per-robot metrics:
Robot 1 | Active ticks: 29 | Moves: 29 | Waits: 0 | Utilization: 93.5%
Robot 2 | Active ticks: 3  | Moves: 3  | Waits: 0 | Utilization: 9.7%
Robot 3 | Active ticks: 31 | Moves: 30 | Waits: 1 | Utilization: 100.0%
```

## Building

### Requirements

* C++20-compatible compiler
* CMake 3.20 or newer
* Git

GoogleTest is fetched automatically by CMake when tests are enabled.

### Configure

From the repository root:

```bash
cmake -S . -B build
```

### Build

```bash
cmake --build build
```

### Run

```bash
./build/warehouse_sim
```

## Testing

The project uses GoogleTest and CTest.

Run all tests with:

```bash
ctest --test-dir build --output-on-failure
```

Or run the GoogleTest executable directly:

```bash
./build/tests/warehouse_tests
```

The current suite contains 13 tests across four areas:

```text
Robot
├── initialization
├── movement
└── state transitions

Pathfinder
├── shortest path
├── obstacle avoidance
└── unreachable goals

Dispatcher
├── shortest reachable robot
├── busy robot exclusion
└── unreachable pickup handling

Simulation
├── queued task completion
├── collision prevention
├── robot failure recovery
└── metrics collection
```

## Continuous Integration

GitHub Actions automatically builds and tests the project on pushes and pull requests to `main`.

The CI pipeline:

```text
Checkout
   |
   v
Configure CMake
   |
   v
Build
   |
   v
Run CTest
```

The workflow runs on Ubuntu, providing an additional Linux build environment alongside local development.

## Design Decisions

### Deterministic ticks instead of one thread per robot

The simulator uses a time-step coordinator instead of assigning an operating-system thread to every robot.

This keeps movement deterministic and makes it possible to evaluate all robot movement proposals before committing the next warehouse state.

### Replanning instead of fixed paths

Robots recalculate routes as the fleet changes.

This increases path-planning work but allows the simulation to react to moving robots rather than following stale routes.

### Separate robot state and robot plan

`Robot` represents the physical robot:

```text
ID
position
state
```

The simulation's internal robot plan represents the currently assigned work:

```text
active task
pickup
dropoff
```

Keeping these concerns separate makes fleet coordination easier to reason about.

### Failed robot removal

Failed robots are moved outside the active grid and marked `Failed`.

This models a robot being removed from operation while still allowing another robot to recover its interrupted task.

## Current Scope

The simulator intentionally focuses on deterministic grid-based fleet coordination.

It does not currently model:

* continuous physical motion
* robot acceleration or turning radius
* battery consumption
* charging stations
* probabilistic sensor input
* physical manipulation
* persistent map storage
* real warehouse hardware interfaces

These constraints keep the project focused on path planning, task scheduling, coordination, failure recovery, and simulation behavior.
