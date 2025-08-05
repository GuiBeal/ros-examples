import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    robot_description_path = get_package_share_path("my_robot_description")
    robot_bringup_path = get_package_share_path("my_robot_bringup")

    urdf_path = LaunchConfiguration("urdf_path")
    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=os.path.join(
            robot_description_path,
            "urdf",
            "my_robot.urdf.xacro",
        ),
        description="Path to robot URDF file",
    )

    rviz_config_path = LaunchConfiguration("rviz_config_path")
    rviz_config_path_arg = DeclareLaunchArgument(
        "rviz_config_path",
        default_value=os.path.join(robot_description_path, "config", "my_robot.rviz"),
        description="Path to rviz config file",
    )

    controllers_config_path = LaunchConfiguration("controllers_config_path")
    controllers_config_path_arg = DeclareLaunchArgument(
        "controllers_config_path",
        default_value=os.path.join(
            robot_bringup_path, "config", "my_robot_controllers.yaml"
        ),
        description="Path to controllers config file",
    )

    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_config_path],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    arm_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_joints_controller"],
    )

    rviz2_node = Node(
        package="rviz2", executable="rviz2", arguments=["-d", rviz_config_path]
    )

    return LaunchDescription(
        [
            urdf_path_arg,
            rviz_config_path_arg,
            controllers_config_path_arg,
            robot_state_publisher_node,
            control_node,
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
            arm_joint_controller_spawner,
            rviz2_node,
        ]
    )
