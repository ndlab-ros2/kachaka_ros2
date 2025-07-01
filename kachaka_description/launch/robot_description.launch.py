#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    description_package_path = get_package_share_path('kachaka_description')
    default_model_path = description_package_path / 'robot/kachaka.urdf.xacro'
    default_rviz_config_path = description_package_path / 'rviz/kachaka.rviz'

    gui_arg = DeclareLaunchArgument(
        name='gui', 
        default_value='true', 
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui')
    
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file')

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="kachaka",
        description="Namespace for the robot state publisher",
    )

    frame_prefix_arg = DeclareLaunchArgument(
        "frame_prefix",
        default_value=EnvironmentVariable("FRAME_PREFIX", default_value = "" ),
        description="Frame prefix for the robot state publisher",
    )

    namespace = LaunchConfiguration("namespace")
    frame_prefix = LaunchConfiguration("frame_prefix")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("kachaka_description"),
                    "robot",
                    "kachaka.urdf.xacro",
                ]
            ),
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_content,
                "frame_prefix": frame_prefix,
            }
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        namespace_arg, 
        frame_prefix_arg, 
        robot_state_publisher_node,
        rviz_node
        ]
    )
