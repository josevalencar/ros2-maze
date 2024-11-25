# ROS2 Maze Solver

![culling games](/lobotomy_kaisen/suksuk.jpg)

## Overview

This repository contains a ROS2 package that implements two approaches to solve a maze using a robotic agent and a service-client workflow:

1. **Reactive Solver**: Navigates the maze using local sensor data without prior knowledge of the maze layout.
2. **Mapping Solver**: Obtains the full maze map, computes the optimal path using the A* algorithm, and navigates accordingly.

This project builds upon the foundational work provided by my professor's repository, **Culling Games**, which uses Pygame to visualize the maze environment. The original codebase can be found here: [Culling Games Repository](https://github.com/rmnicola/culling_games).

## How It Workss

1. **Reactive Solver (`reactive`)**:
   - Continuously queries the robot's sensors to make movement decisions.
   - Maintains a set of visited positions to prevent revisiting.
   - Performs backtracking when it reaches dead ends.

   ![demo gif r](https://i.giphy.com/media/v1.Y2lkPTc5MGI3NjExa3c2b3Y3cjkxN3g2eHpyMWd1c2Q3eHY4Z2k5YTBxNTkxZG5pOWV2NCZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/OtOXTplsPJGkYAg89s/giphy.gif)

2. **Mapping Solver (`mapping`)**:
   - Requests the full maze map from the `get_map` service.
   - Parses the map to identify free spaces, obstacles, the robot's position (`'r'`), and the target (`'t'`).
   - Uses the A* algorithm to compute the optimal path.
   - Navigates the robot along the computed path.

   ![demo gif m](https://i.giphy.com/media/v1.Y2lkPTc5MGI3NjExOHlwMTNtc3Z4ZjB3dmc2c2oxb3VjdzExampyeWtkb3dkODJmNndhbCZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/z2Yy5WU1Y5U50mYDVI/giphy.gif)

## How to build and run

### Prerequisites

- **ROS2 (e.g., ROS2 Humble)**
- **Colcon**: For building ROS2 packages.
- **Python 3**

### Clone the repositories

1. **Clone the repository**

```bash
git clone https://github.com/josevalencar/ros2-maze.git
```

### Build the Packages

1. **Navigate to the ros2 workspace**

```bash
cd ros2-maze
```

2. **Build the Workspace**

```bash
colcon build
```

3. **Source the Workspace**

```bash
source install/local_setup.bash
```

### Running the Maze Environment

1. Before running the solver nodes, you need to start the maze environment provided by the Culling Games package:

```bash
ros2 run cg maze
```

2. You can also edit the maze using: 
```bash
ros2 run cg edit
```

### Running the Solver Nodes

#### Reactive Solver

```bash
ros2 run pysolver reactive
```

#### Mapping Solver

```bash
ros2 run pysolver mapping
```

Note: Ensure that the cg package (from Culling Games) is running, as it provides the necessary services (move_command, get_map) and the maze environment for the solver nodes.

## References

Maze Solving Algorithms for Micro Mouse: [Article](https://swati-mishra.com/wp-content/uploads/2020/02/04725791.pdf)

Python Documentation on heapq: [heapq — Heap queue algorithm](https://docs.python.org/3/library/heapq.html)

A* Algorithm Explanation: [A* Search Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm)