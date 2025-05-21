#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API for UR5e.
"""

import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=["scaled_joint_trajectory_controller"])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main():
    # Wait to ensure RViz and other ROS nodes are fully up
    print("Waiting 5 seconds for RViz and MoveGroup to initialize...")
    time.sleep(10)

    
    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    ur5e = MoveItPy(node_name="moveit_py")
    ur_arm = ur5e.get_planning_component("ur_arm")
    logger.info("MoveItPy instance created")

    ###########################################################################
    # Plan 1 – From current state to custom pose
    ###########################################################################
    from geometry_msgs.msg import PoseStamped

    ur_arm.set_start_state_to_current_state()

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "ur5e_base_link"
    pose_goal.pose.orientation.w = 0.7071
    pose_goal.pose.orientation.x = 0.0
    pose_goal.pose.orientation.y = -0.7071
    pose_goal.pose.orientation.z = 0.0
    # Define the pose goal

    pose_goal.pose.position.x = -0.2
    pose_goal.pose.position.y = 0.2
    pose_goal.pose.position.z = 0.8
    logger.info("Planning to custom pose goal")
    ur_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="ur5e_tool0")

    # Create multi-pipeline plan request parameters using configurations from YAML
    pipeline_names = ["ompl", "pilz_industrial_motion_planner", "chomp"]
    multi_plan_parameters = MultiPipelinePlanRequestParameters(ur5e, pipeline_names)

    plan_and_execute(ur5e, ur_arm, logger, multi_plan_parameters=multi_plan_parameters, sleep_time=3.0)

    ###########################################################################
    # Plan 2 – Move to 'home' state
    ###########################################################################
    ur_arm.set_start_state_to_current_state()
    logger.info("Planning to 'home' position")
    ur_arm.set_goal_state(configuration_name="home")

    plan_and_execute(ur5e, ur_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 3 – Move to 'extended' state
    ###########################################################################
    ur_arm.set_start_state_to_current_state()
    logger.info("Planning to 'extended' position")
    ur_arm.set_goal_state(configuration_name="extended")

    plan_and_execute(ur5e, ur_arm, logger, sleep_time=3.0)


if __name__ == "__main__":
    main()
