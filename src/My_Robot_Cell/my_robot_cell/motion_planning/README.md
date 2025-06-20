# motion_planning

The `motion_planning` package contains a custom Python-based API for high-level motion planning and execution. It defines which planners to use (such as OMPL and Pilz) and configures parameters like planning time, number of attempts, and goal tolerances.

## Key Features

- **Planner setup**  
  The API selects and configures planners (e.g., OMPL’s RRT-Connect and Pilz’s LIN planner) and their parameters before each motion request.

- **Sequential execution with feedback**  
  Tasks are executed one goal at a time, ensuring that each motion completes before starting the next. Built-in result feedback handling improves reliability.

- **Example scripts**  
  Example Python scripts (under `scripts/`) demonstrate common tasks. For instance:
  - `pick_and_place.py` — shows how to use the API for pick-and-place operations.
  - `publisher_node.py` — demonstrates publishing motion commands via ROS2 topics.

## Purpose

This package abstracts the complexity of ROS2-MoveIt2 interaction. A user can import the API or run the provided scripts to perform coordinated pick-and-place operations on the dual-arm system.
