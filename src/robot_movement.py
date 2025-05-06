#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes, RobotState, Constraints, JointConstraint, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from shape_msgs.msg import SolidPrimitive
from math import pi

class RobotMovement(Node):
    def __init__(self):
        super().__init__('robot_movement')
        
        # Create action client for move group
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Wait for the action server
        while not self.move_group_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for move group action server...')
        
        self.get_logger().info('Robot movement node initialized')

    def move_to_position(self, position, orientation, time_from_start=5.0):
        """
        Move robot to specified cartesian position
        
        Args:
            position (list): [x, y, z] position in meters
            orientation (list): [rx, ry, rz] orientation in radians (roll, pitch, yaw)
            time_from_start (float): Time in seconds to reach the target position
        """
        try:
            # Create target pose
            target_pose = Pose()
            target_pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
            
            # Convert Euler angles (rx, ry, rz) to quaternion
            from math import sin, cos
            rx, ry, rz = orientation
            
            # Simple conversion from Euler angles to quaternion (assuming XYZ order)
            cy = cos(rz * 0.5)
            sy = sin(rz * 0.5)
            cp = cos(ry * 0.5)
            sp = sin(ry * 0.5)
            cr = cos(rx * 0.5)
            sr = sin(rx * 0.5)

            target_pose.orientation = Quaternion(
                x=sr * cp * cy - cr * sp * sy,
                y=cr * sp * cy + sr * cp * sy,
                z=cr * cp * sy - sr * sp * cy,
                w=cr * cp * cy + sr * sp * sy
            )
            
            # Create goal message
            goal_msg = MoveGroup.Goal()
            
            # Set up position constraint
            pos_constraint = PositionConstraint()
            pos_constraint.header.frame_id = "ur5e_base_link"
            pos_constraint.link_name = "ur5e_tool0"
            
            # Create a solid primitive for the constraint region
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions = [0.1]  # 10cm radius sphere
            
            pos_constraint.constraint_region.primitives = [primitive]
            pos_constraint.constraint_region.primitive_poses = [target_pose]
            pos_constraint.weight = 1.0
            
            # Set up orientation constraint
            orient_constraint = OrientationConstraint()
            orient_constraint.header.frame_id = "ur5e_base_link"
            orient_constraint.link_name = "ur5e_tool0"
            orient_constraint.orientation = target_pose.orientation
            orient_constraint.absolute_x_axis_tolerance = 0.1
            orient_constraint.absolute_y_axis_tolerance = 0.1
            orient_constraint.absolute_z_axis_tolerance = 0.1
            orient_constraint.weight = 1.0
            
            # Set up constraints
            constraints = Constraints()
            constraints.position_constraints = [pos_constraint]
            constraints.orientation_constraints = [orient_constraint]
            
            # Set up the goal
            goal_msg.request.group_name = "ur_arm"
            goal_msg.request.num_planning_attempts = 10
            goal_msg.request.allowed_planning_time = time_from_start
            goal_msg.request.max_velocity_scaling_factor = 0.1
            goal_msg.request.max_acceleration_scaling_factor = 0.1
            goal_msg.request.goal_constraints = [constraints]
            
            # Send goal
            self.get_logger().info('Sending goal...')
            future = self.move_group_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self.get_logger().error('Goal rejected')
                    return False
                
                self.get_logger().info('Goal accepted')
                
                # Wait for the result
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                
                if result_future.result() is not None:
                    result = result_future.result().result
                    self.get_logger().info('Result: {0}'.format(result.error_code))
                    return result.error_code == 0
                else:
                    self.get_logger().error('Exception while calling service: {0}'.format(result_future.exception()))
                    return False
            else:
                self.get_logger().error('Failed to send goal')
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error in movement: {str(e)}")
            return False

def main(args=None):
    if rclpy.ok():
        rclpy.try_shutdown()
    
    rclpy.init(args=args)
    
    try:
        robot_movement = RobotMovement()
        
        # Example usage
        # Move to home position (in front of robot, slightly above base)
        home_position = [0.4, 0.0, 0.4]  # x, y, z in meters
        home_orientation = [0.0, pi, 0.0]  # roll, pitch, yaw in radians
        robot_movement.get_logger().info('Moving to home position...')
        robot_movement.move_to_position(home_position, home_orientation)
        
        # Move to position 1 (to the right)
        position1 = [0.4, 0.3, 0.4]  # x, y, z in meters
        orientation1 = [0.0, pi, 0.0]  # roll, pitch, yaw in radians
        robot_movement.get_logger().info('Moving to position 1...')
        robot_movement.move_to_position(position1, orientation1)
        
        # Move to position 2 (to the left)
        position2 = [0.4, -0.3, 0.4]  # x, y, z in meters
        orientation2 = [0.0, pi, 0.0]  # roll, pitch, yaw in radians
        robot_movement.get_logger().info('Moving to position 2...')
        robot_movement.move_to_position(position2, orientation2)
        
        # Return to home position
        robot_movement.get_logger().info('Returning to home position...')
        robot_movement.move_to_position(home_position, home_orientation)
        
        rclpy.spin(robot_movement)
    except KeyboardInterrupt:
        pass
    finally:
        if 'robot_movement' in locals():
            robot_movement.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main() 