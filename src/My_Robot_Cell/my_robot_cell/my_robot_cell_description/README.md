# my_robot_cell_description

The `my_robot_cell_description` package provides the URDF (Unified Robot Description Format) model of the entire robot cell. This virtual representation includes two collaborative robotic arms (two UR5e robots) with their grippers (modeled as simple boxes), the assembly table, the physical wall, and safety guards around the workspace.

These elements match the real lab setup and ensure that simulations and visualizations (e.g., in RViz) accurately reflect the hardware.

## Key Features

- **Two UR5e arms with grippers**  
  Each gripper is represented by a box model in the URDF.

- **Workspace features**  
  The model includes the assembly table, the surrounding real wall, and safety fences around the human workspace.

- **Gripper meshes**  
  Detailed OnRobot 2FG7 gripper files (STL and XACRO) are included but not used in the URDF.



 The URDF can be visualized using RViz to verify that all parts (robots, objects, and environment) are correctly placed and scaled.
