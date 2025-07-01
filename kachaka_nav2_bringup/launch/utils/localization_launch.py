# Copyright (C) 2025 CyberAgent AI Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import SetParameter
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    kachaka_nav2_bringup_dir = get_package_share_directory('kachaka_nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    rviz_config_file = os.path.join(
        kachaka_nav2_bringup_dir,
        'rviz',
        'kachaka-nav.rviz')
    
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)

    lifecycle_nodes = [
        'map_server', 
        'amcl'
        ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("/odom", "/kachaka/wheel_odometry/wheel_odometry"),
        ("/scan", "/kachaka/lidar/scan"),
        ("/map", "/kachaka/mapping/map"),
        ("/map_updates", "/kachaka/mapping/map_updates"),
    ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time,
                           'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True,
    )
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', 
        default_value='', 
        description='Top-level namespace'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', 
        default_value='', 
        description='Full path to map yaml file to load'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(kachaka_nav2_bringup_dir, 
                                   'params', 
                                   'localization_param.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack'
        )
    
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', 
        default_value='False',
        description='Use composed bringup if True'
        )
    
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    load_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration('map'), "' == ''"])
                ),
                #起動するパッケージの指定
                package='nav2_map_server', 
                #実行ファイルの指定
                executable='map_server',
                #このノードの名前の指定
                name='map_server',
                #ノードの出力先をterminalに指定
                output='screen',
                #ノードがクラッシュ・終了したときに自動再起動するかどうかの設定
                respawn=use_respawn,
                #ノードが終了した後、再起動するまで待つ時間の設定
                respawn_delay=2.0,
                #このノードに与えるパラメータの設定
                parameters=[configured_params],
                #ros2コマンド経由でノードに渡す追加引数の設定
                arguments=['--ros-args', '--log-level', log_level],
                #リマッピングの指定
                remappings=remappings,
            ),
            Node(
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration('map'), "' != ''"])
                ),
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'yaml_filename': map_yaml_file}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            #ロボットの自己位置を地図上で推定するノード
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            #ライフサイクルマネージャ(map_serverやamclの状態を管理するもの)のノード
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
            ),
        ],
    )

    # rviz2_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_file],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the localization nodes
    ld.add_action(load_nodes)
    # ld.add_action(rviz2_node)

    return ld
