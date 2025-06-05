#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time


class PoseGoalPublisher(Node):
    def __init__(self):
        super().__init__('pose_goal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.pose_index = 0
        self.poses = self.generate_poses()
        self.get_logger().info("PoseGoalPublisher initialized.")

    def generate_poses(self):
        """Define a list of target poses with positions and orientations."""
        poses = []

        def make_pose(x, y, z, ox, oy, oz, ow):
            pose = PoseStamped()
            pose.header.frame_id = 'ur5e_base_link'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.x = ox
            pose.pose.orientation.y = oy
            pose.pose.orientation.z = oz
            pose.pose.orientation.w = ow
            return pose



        # Pose 1: Front position with upward orientation
        poses.append(make_pose(0.3, 0.2, 0.5, 0.0, 0.0, 0.0, 1.0))
        
        # Pose 2: Left position with slight tilt
        poses.append(make_pose(-0.3, 0.0, 0.5, 0.0, 0.0, 0.707, 0.707))
        
        # Pose 3: Right position with different orientation
        poses.append(make_pose(-0.3, 0.2, 0.5, 0.0, 0.0, -0.707, 0.707))
        
        # Pose 4: Center position with upward orientation
        poses.append(make_pose(0.0, 0.3, 0.6, 0.0, 0.0, 0.0, 1.0))
        
        # Pose 5: Final position with neutral orientation
        poses.append(make_pose(-0.4, 0.4, 0.5, 0.0, 0.0, 0.0, 1.0))

        return poses

    def timer_callback(self):
        if self.pose_index < len(self.poses):
            pose = self.poses[self.pose_index]
            pose.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(pose)
            self.get_logger().info(f"Published pose {self.pose_index + 1}")
            self.pose_index += 1
        else:
            self.get_logger().info("All poses published. Stopping timer.")
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = PoseGoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
