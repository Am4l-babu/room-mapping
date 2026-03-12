"""
============================================================================
gazebo_launch.py — Main Launch File: Gazebo Harmonic + Robot + Bridge
============================================================================

Starts the full simulation stack using Gazebo Harmonic (gz-sim):

  1. gz_sim         — Gazebo Harmonic physics + rendering
  2. robot_state_publisher — publishes TF from URDF
  3. spawn robot    — places robot into the running Gazebo world
  4. ros_gz_bridge  — bridges gz topics ↔ ROS2 topics:
       /cmd_vel    (ROS → gz)  geometry_msgs/Twist
       /odom       (gz → ROS)  nav_msgs/Odometry
       /scan       (gz → ROS)  sensor_msgs/LaserScan
       /imu        (gz → ROS)  sensor_msgs/Imu
       /tf         (gz → ROS)  tf2_msgs/TFMessage
       /joint_states(gz → ROS) sensor_msgs/JointState
       /clock      (gz → ROS)  rosgraph_msgs/Clock

USAGE:
  ros2 launch room_mapping_robot gazebo_launch.py

NOTE: Requires Gazebo Harmonic (gz-sim 8.x) with ros_gz_sim + ros_gz_bridge.
      Classic gazebo_ros is NOT used.
============================================================================
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Gazebo Harmonic simulation."""

    # ---- Package paths ----
    pkg_share = get_package_share_directory('room_mapping_robot')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # ---- File paths ----
    urdf_file  = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'room.world')

    # ---- Process xacro → URDF XML ----
    robot_description_content = xacro.process_file(urdf_file).toxml()

    # ---- Launch Arguments ----
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock'
    )

    # ---- Robot State Publisher ----
    # Reads URDF, publishes static TFs and /robot_description for RViz
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

    # ---- Gazebo Harmonic (gz_sim.launch.py from ros_gz_sim) ----
    # -r: run simulation immediately (don't pause at startup)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ---- Spawn robot into Gazebo ----
    # ros_gz_sim 'create' reads URDF from /robot_description ROS2 topic
    # Delay 3s to ensure Gazebo is ready before spawning
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-name', 'room_mapping_robot',
                    '-topic', '/robot_description',
                    '-x', '1.0',
                    '-y', '1.0',
                    '-z', '0.05',
                    '-Y', '0.0',
                ],
            )
        ],
    )

    # ---- ROS-GZ Bridge ----
    # Converts Gazebo Harmonic gz-transport topics ↔ ROS2 topics.
    # Bridge direction syntax:
    #   [  = gz publishes → ROS subscribes  (gz → ROS)
    #   ]  = ROS publishes → gz subscribes  (ROS → gz)
    #   @  = bidirectional
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            # Velocity commands: ROS Nav2/teleop → Gazebo robot
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Odometry: Gazebo diff_drive → ROS (SLAM, Nav2)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # LiDAR scan: Gazebo gpu_lidar → ROS SLAM Toolbox
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # IMU: Gazebo imu sensor → ROS
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # TF: Gazebo diff_drive odom→base_footprint → ROS TF tree
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Joint states: Gazebo joint_state_publisher → ROS RSP
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Simulation clock: Gazebo → ROS (required for use_sim_time)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ---- Assemble Launch Description ----
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
        gz_sim,
        spawn_robot,
        bridge,
    ])
