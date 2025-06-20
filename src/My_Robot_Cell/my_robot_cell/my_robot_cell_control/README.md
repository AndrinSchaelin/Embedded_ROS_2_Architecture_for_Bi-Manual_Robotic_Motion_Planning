# my_robot_cell_control

The `my_robot_cell_control` package provides the ROS2 controllers and launch scripts needed to operate both UR5e robots. It includes YAML configuration files defining the joint trajectory and controller parameters for each robot arm, as well as base calibration settings.

## Main Features

- **Controller configurations**  
  YAML files (`dual_robot_controllers.yaml`, `409_controllers.yaml`, etc.) that specify the ROS2 control interfaces for both UR5e arms and the old controller file of the 409 as reference.

- **Launch files**  
  Launch scripts for starting the robots in dual- or single-arm mode (`dual_robot.launch.py`, `single_robot.launch.py`, etc.). These bring up the necessary ROS2 nodes for real hardware or simulated execution. At the moment only the dual_robot.launch.py is working properly.

- **Calibration data**  
  Parameters in `my_robot_calibration.yaml` ensure that the robot base coordinate frames are correctly aligned relative to the workbench.

## Usage

By using this package, users can start all necessary controllers and robot drivers with a single command. For example, launching:

```bash
ros2 launch my_robot_cell_control dual_robot.launch.py use_mock_hardware:=true
