#!/bin/bash

# Pose 1
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'ur5e_base_link'}, pose: {position: {x: 0.4, y: 0.2, z: 0.6}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
sleep 5

# Pose 2
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'ur5e_base_link'}, pose: {position: {x: -0.5, y: 0.5, z: 0.2}, orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}}}"
sleep 5

# Pose 3
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'ur5e_base_link'}, pose: {position: {x: 0.3, y: 0.0, z: 0.8}, orientation: {x: 0.7071, y: 0.0, z: 0.0, w: 0.7071}}}"
sleep 5

# Pose 4
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'ur5e_base_link'}, pose: {position: {x: -0.6, y: 0.3, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}}}"
sleep 5

# Pose 5
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'ur5e_base_link'}, pose: {position: {x: 0.4, y: 0.3, z: 0.65}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
