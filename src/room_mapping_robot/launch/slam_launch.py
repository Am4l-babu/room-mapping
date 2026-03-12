"""
============================================================================
slam_launch.py — SLAM Toolbox Launch File for Room Mapping
============================================================================

This launch file starts SLAM Toolbox in ONLINE ASYNC mode, which is the
recommended mode for real-time mapping with a moving robot.

SLAM TOOLBOX MODES:
  1. online_async  — Process scans as they arrive, non-blocking (USED HERE)
  2. online_sync   — Process every scan, may lag with slow CPUs
  3. offline        — Process a bag file (for post-processing)
  4. localization   — Use an existing map, don't update it

WHAT SLAM TOOLBOX DOES:
  1. Subscribes to /scan (LaserScan from LiDAR)
  2. Uses /odom for initial pose estimates between scans
  3. Performs scan matching (correlative scan matcher)
  4. Builds an occupancy grid map
  5. Publishes /map (nav_msgs/OccupancyGrid)
  6. Broadcasts map → odom TF transform
  7. Performs loop closure detection

AFTER THIS LAUNCH, THE COMPLETE TF TREE IS:
  map → odom → base_footprint → base_link → {lidar_link, imu_link}
  
  Where:
  - map → odom            : published by SLAM Toolbox (corrects drift)
  - odom → base_footprint : published by diff_drive Gazebo plugin
  - base_link → children  : published by robot_state_publisher

HOW TO USE:
  $ ros2 launch room_mapping_robot slam_launch.py
  
  Then drive the robot around using teleop_twist_keyboard to build the map.
  View the map in RViz by subscribing to /map topic.

SAVING THE MAP:
  When mapping is complete, save the map:
  $ ros2 run nav2_map_server map_saver_cli -f ~/my_room_map
  
  This saves two files:
  - my_room_map.pgm (image)
  - my_room_map.yaml (metadata)

REAL ROBOT CONNECTION:
  This same launch file works with the real robot!
  As long as the ESP32 publishes /scan and /odom via micro-ROS,
  SLAM Toolbox will map the room identically.
  Just change use_sim_time to false.
============================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for SLAM Toolbox mapping."""

    # ---- Package path ----
    pkg_share = get_package_share_directory('room_mapping_robot')

    # ---- SLAM parameters file ----
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')

    # ---- Launch Arguments ----
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (true for Gazebo, false for real robot)'
    )

    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params_file,
        description='Path to SLAM Toolbox parameter file'
    )

    # ---- SLAM Toolbox Node ----
    # online_async_launch.py from slam_toolbox runs the async SLAM node
    # We configure it with our custom parameters
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

    # ---- RViz2 (optional — can also be launched separately) ----
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'mapping.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_params_arg,
        slam_toolbox_node,
        rviz_node,
    ])
