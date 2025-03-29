import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('environment')
    xacro_file = os.path.join(pkg_share, 'urdf', 'environment.xacro')

    robot_description = Command([
        FindExecutable(name='xacro'), ' ', xacro_file
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'view_robot.rviz')],
            output='screen'
        )
    ])
