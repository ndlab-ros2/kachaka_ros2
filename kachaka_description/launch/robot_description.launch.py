#!/usr/bin/env python3
import os
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


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="kachaka",
        description="Namespace for the robot state publisher",
    )

    frame_prefix_arg = DeclareLaunchArgument(
        "frame_prefix",
        default_value=EnvironmentVariable("FRAME_PREFIX"),
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

    # === 追加: RViz起動ノード ===
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("kachaka_description"),
        "rviz",
        "kachaka.rviz"  # ← 実際のファイル名に合わせて修正
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    front_camera_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='front_camera_view',
        arguments=['/kachaka/front_camera/image_raw/compressed'],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(namespace_arg)
    ld.add_action(frame_prefix_arg)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(front_camera_view_node)  

    return ld
