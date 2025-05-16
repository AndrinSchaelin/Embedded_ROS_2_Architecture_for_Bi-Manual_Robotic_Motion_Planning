from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch, generate_moveit_rviz_launch
import os


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
            default_value="ur5e",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.100",
            description="IP address by which the robot can be reached.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware.",
        )
    )

    # Get configurations
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Include the robot control launch file
    robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("my_robot_cell_control"),
                        "launch",
                        "start_robot.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "use_mock_hardware": use_mock_hardware,
        }.items(),
    )

    # Configure MoveIt (old one without motion planning python api)
    #moveit_config = MoveItConfigsBuilder("my_robot_cell", package_name="my_robot_cell_moveit2_config").to_moveit_configs()

    # Get MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("my_robot_cell", package_name="my_robot_cell_moveit2_config")
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("motion_planning"),
                "config",
                "motion_planning_python_api.yaml"
            )
        )
        .to_moveit_configs()
    )

    # Create the motion planning Python API node
    moveit_py_node = Node(
        name="moveit_py",
        executable="python3",
        arguments=[os.path.join(get_package_share_directory("motion_planning"), "scripts", "motion_planning_python_api.py")],
        output="screen",
        parameters=[moveit_config.to_dict()]
    )

    # Include move_group launch
    move_group = generate_move_group_launch(moveit_config)

    # Include MoveIt Rviz launch
    moveit_rviz = generate_moveit_rviz_launch(moveit_config)

    return LaunchDescription(
        declared_arguments
        + [
            robot_control,
            move_group,
            moveit_rviz,
            moveit_py_node,
        ]
    ) 
