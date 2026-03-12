"""
============================================================================
navigation_launch.py — Full Nav2 Navigation + SLAM Launch File
============================================================================

This launch file starts the COMPLETE autonomous navigation stack:

  1. Robot State Publisher  — URDF → TF transforms
  2. Gazebo (optional)      — if not already running
  3. SLAM Toolbox           — live mapping (/map + map→odom TF)
  4. Nav2 Stack             — full navigation (planner + controller + behaviors)
  5. RViz2                  — visualization with Nav2 panels

WHAT THIS ENABLES:
  - Autonomous point-to-point navigation (click goal in RViz)
  - Path planning around obstacles
  - Recovery behaviors (spin, backup) when stuck
  - Simultaneous mapping + navigation (explore while building map)

HOW TO USE:
  # Terminal 1: Start Gazebo with robot
  $ ros2 launch room_mapping_robot gazebo_launch.py

  # Terminal 2: Start Nav2 + SLAM + RViz
  $ ros2 launch room_mapping_robot navigation_launch.py

  # Terminal 3 (optional): Start autonomous explorer
  $ ros2 run room_mapping_robot autonomous_explorer.py

  You can also click "Nav2 Goal" in RViz to send manual goals.

ARCHITECTURE:
  ┌──────────┐    ┌───────────┐    ┌──────────────┐    ┌────────┐
  │  Explorer │───→│ BT Nav    │───→│   Planner    │───→│  Path  │
  │  (goals)  │    │ (behavior │    │  (NavFn A*)  │    │        │
  └──────────┘    │  tree)    │    └──────────────┘    └───┬────┘
                  └─────┬─────┘                            │
                        │                                  ▼
                        │           ┌──────────────┐    ┌────────┐
                        └──────────→│  Controller  │───→│cmd_vel │
                        (if stuck)  │  (DWB local) │    │        │
                        │           └──────────────┘    └────────┘
                  ┌─────▼─────┐
                  │ Behaviors │
                  │ (spin,    │
                  │  backup)  │
                  └───────────┘

TF TREE (complete):
  map → odom → base_footprint → base_link → {lidar_link, imu_link, wheels}
  (map→odom from SLAM, odom→base_footprint from diff_drive)

REAL ROBOT:
  Same launch works with real ESP32 robot! Just change:
    $ ros2 launch room_mapping_robot navigation_launch.py use_sim_time:=false
============================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    """Generate launch description for Nav2 navigation + SLAM."""

    # ---- Package paths ----
    pkg_share = get_package_share_directory('room_mapping_robot')

    # ---- File paths ----
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'nav2_rviz.rviz')

    # ---- Launch Arguments ----
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock'
    )

    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=nav2_params_file,
        description='Path to Nav2 parameter file'
    )

    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params_file,
        description='Path to SLAM Toolbox parameter file'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 with Nav2 config'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 lifecycle nodes'
    )

    # ---- SLAM Toolbox ----
    # Runs SLAM in online_async mode for simultaneous mapping + navigation
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # ---- Nav2 Controller Server ----
    # DWB local controller: follows planned paths, avoids obstacles
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            LaunchConfiguration('nav2_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # ---- Nav2 Planner Server ----
    # NavFn A* global planner: computes paths on the costmap
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            LaunchConfiguration('nav2_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # ---- Nav2 Behavior Server ----
    # Recovery behaviors: spin, backup, wait
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            LaunchConfiguration('nav2_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # ---- Nav2 BT Navigator ----
    # Behavior tree navigator: orchestrates planning, control, and recovery
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            LaunchConfiguration('nav2_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # ---- Nav2 Waypoint Follower ----
    # Follows a sequence of waypoints (used by autonomous explorer)
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            LaunchConfiguration('nav2_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # ---- Nav2 Velocity Smoother ----
    # Smooths cmd_vel output for gentler motor control
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            LaunchConfiguration('nav2_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),         # Input from Nav2 controller
            ('cmd_vel_smoothed', 'cmd_vel'),    # Output to robot
        ],
    )

    # ---- Nav2 Lifecycle Manager ----
    # Manages the lifecycle of all Nav2 nodes:
    # unconfigured → inactive → active
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            LaunchConfiguration('nav2_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': LaunchConfiguration('autostart'),
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                ],
            },
        ],
    )

    # ---- RViz2 ----
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # ---- Assemble Launch Description ----
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        nav2_params_arg,
        slam_params_arg,
        launch_rviz_arg,
        autostart_arg,

        # SLAM
        slam_toolbox_node,

        # Nav2 nodes
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,

        # Visualization
        rviz_node,
    ])
