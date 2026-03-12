"""
============================================================================
spawn_robot.launch.py — Standalone Robot Spawner for Gazebo Harmonic
============================================================================

Spawns the robot into an already-running Gazebo Harmonic instance.
Uses ros_gz_sim 'create' executable (NOT gazebo_ros spawn_entity.py).

USAGE:
  # First, start Gazebo Harmonic manually:
  $ gz sim <path_to_room.world> -r

  # Then spawn the robot:
  $ ros2 launch room_mapping_robot spawn_robot.launch.py

PARAMETERS:
  x, y, z    — spawn position (default: 1.0, 1.0, 0.05)
  yaw        — initial heading (default: 0.0 radians)
  robot_name — entity name in Gazebo (default: room_mapping_robot)
============================================================================
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description to spawn robot into running Gazebo Harmonic."""

    # ---- Package and file paths ----
    pkg_share = get_package_share_directory('room_mapping_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    # ---- Process xacro → URDF ----
    robot_description_content = xacro.process_file(urdf_file).toxml()

    # ---- Launch Arguments ----
    declare_x = DeclareLaunchArgument('x', default_value='1.0',
                                       description='X spawn position')
    declare_y = DeclareLaunchArgument('y', default_value='1.0',
                                       description='Y spawn position')
    declare_z = DeclareLaunchArgument('z', default_value='0.05',
                                       description='Z spawn position')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0',
                                         description='Initial yaw angle')
    declare_name = DeclareLaunchArgument(
        'robot_name', default_value='room_mapping_robot',
        description='Robot entity name in Gazebo'
    )

    # ---- Robot State Publisher ----
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }],
    )

    # ---- Spawn Robot via ros_gz_sim create ----
    # Reads URDF from /robot_description topic published above
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-topic', '/robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    return LaunchDescription([
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        declare_name,
        robot_state_publisher_node,
        spawn_robot,
    ])
