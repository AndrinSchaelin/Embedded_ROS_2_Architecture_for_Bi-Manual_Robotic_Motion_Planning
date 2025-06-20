# moveit_config

The `moveit_config` package contains the MoveIt2 configuration for the dual-arm robot cell. Using the final URDF from the `my_robot_cell_description` package, this configuration defines the planning groups and kinematic solvers for both arms, as well as all motion-planning parameters.

## Key Contents

- **Planning groups and kinematics**  
  Definitions of each arm’s planning group and kinematic solver settings, based on the URDF’s joint structure.

- **Joint limit constraints**  
  YAML files (`joint_limits.yaml`, etc.) that specify each joint’s motion limits (position, velocity, acceleration).

- **Collision settings**  
  Custom collision matrix and Cartesian limit files (e.g., `pilz_cartesian_limits.yaml`) to ensure safe planning between the two arms and surrounding objects.

- **Controller interface**  
  Configuration for MoveIt2 controllers (`moveit_controllers.yaml`, `ros2_controllers.yaml`), aligning MoveIt’s commands with the ROS2 control interfaces.

## Notes

These files were generated with the MoveIt2 Setup Assistant and then hand-tuned for the bi-manual setup. With this configuration, MoveIt2 can plan and execute coordinated, collision-free motions for both robot arms in the shared workspace.
