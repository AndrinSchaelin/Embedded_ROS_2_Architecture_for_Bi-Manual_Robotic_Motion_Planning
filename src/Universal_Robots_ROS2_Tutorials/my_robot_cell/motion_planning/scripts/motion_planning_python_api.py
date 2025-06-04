#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters


class PoseGoalPlanner(Node):
    def __init__(self):
        super().__init__('pose_goal_listener')
        self.logger = get_logger('moveit_py.pose_goal')

        # Initialize MoveItPy (creates its own internal node)
        self.moveit = MoveItPy(node_name="moveit_py")
        self.robot_model = self.moveit.get_robot_model()
        self.robot = self.moveit
        self.arm = self.moveit.get_planning_component("ur_arm")

        self.logger.info("MoveItPy and PlanningComponent initialized.")

        # Subscribe to incoming pose goals
        self.subscription = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        self.logger.info(f"Received new target pose in frame '{msg.header.frame_id}'")

        # Plan to the received pose
        try:
            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(pose_stamped_msg=msg, pose_link="ur5e_tool0")

            # Use multiple planners
            multi_planners = MultiPipelinePlanRequestParameters(
                self.moveit,
                ["ompl_rrtc", "pilz_lin", "chomp_planner"]  # must match those in your YAML config
            )

            self.logger.info("Planning with multiple pipelines...")
            plan_result = self.arm.plan(multi_plan_parameters=multi_planners)

            if plan_result:
                self.logger.info("Planning successful. Executing trajectory...")
                self.robot.execute(plan_result.trajectory, controllers=[])
            else:
                self.logger.error("Planning failed.")
        except Exception as e:
            self.logger.error(f"Exception during planning or execution: {e}")


def main():
    rclpy.init()
    node = PoseGoalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
