#!/usr/bin/env python3
"""
============================================================================
autonomous_explorer.py — Frontier-Based Autonomous Room Explorer
============================================================================

This node makes the robot AUTONOMOUSLY explore and map the room using
frontier-based exploration with Nav2 navigation.

WHAT IS FRONTIER EXPLORATION?
  A "frontier" is the boundary between KNOWN FREE space and UNKNOWN space
  on the occupancy grid map. By repeatedly navigating to frontiers, the
  robot systematically explores and maps the entire room.

  Unknown (grey) ──┐
                   │ ← FRONTIER (boundary)
  Free (white)  ───┘

ALGORITHM:
  1. Subscribe to /map (OccupancyGrid from SLAM Toolbox)
  2. Find all frontier cells (free cells adjacent to unknown cells)
  3. Cluster frontiers into groups (nearby frontier cells)
  4. Pick the best frontier cluster:
     - Close enough to reach efficiently
     - Large enough to be worth exploring (not tiny gaps)
     - Prefer closer frontiers to reduce travel time
  5. Send a navigation goal to Nav2 (NavigateToPose action)
  6. Wait for the robot to reach the goal (or fail/timeout)
  7. Repeat from step 2 until no frontiers remain
  8. When no frontiers → room is fully mapped!

USAGE:
  # First, start Gazebo + Nav2:
  $ ros2 launch room_mapping_robot gazebo_launch.py
  $ ros2 launch room_mapping_robot navigation_launch.py

  # Then start autonomous exploration:
  $ ros2 run room_mapping_robot autonomous_explorer.py

  Watch RViz as the robot autonomously navigates to unexplored areas
  and builds the complete room map!

PARAMETERS:
  - min_frontier_size   : Minimum frontier cluster size (ignore tiny gaps)
  - exploration_radius  : How far to look for frontiers (meters)
  - goal_timeout        : Max time to reach a goal before retrying (seconds)
  - obstacle_threshold  : OccupancyGrid value above which = obstacle
  - free_threshold      : OccupancyGrid value below which = free space

TOPICS:
  Subscribed:
    /map              — OccupancyGrid from SLAM Toolbox
    /odom             — Robot odometry (for current position)
  Action Client:
    /navigate_to_pose — Nav2 navigation action

REAL ROBOT:
  This same explorer works with the real ESP32 robot!
  The algorithm doesn't care where /map and /odom come from.
  Just launch with use_sim_time:=false.
============================================================================
"""

import math
import time
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# ===========================================================================
# CONSTANTS
# ===========================================================================

# OccupancyGrid values
UNKNOWN = -1        # Unknown space (grey in RViz)
FREE = 0            # Free space (white in RViz), values 0..free_threshold
OCCUPIED = 100      # Occupied (black in RViz), values obstacle_threshold..100

# Exploration parameters (defaults, can be overridden via ROS params)
DEFAULT_MIN_FRONTIER_SIZE = 3       # Minimum cells in a frontier cluster
DEFAULT_MAX_GOAL_DISTANCE = 3.0     # Max distance to frontier goal (meters)
DEFAULT_MIN_GOAL_DISTANCE = 0.3     # Min distance (don't pick goals on top of robot)
DEFAULT_GOAL_TIMEOUT = 45.0         # Seconds to reach a goal before giving up
DEFAULT_FREE_THRESHOLD = 25         # Below this = free space
DEFAULT_OBSTACLE_THRESHOLD = 65     # Above this = obstacle
DEFAULT_EXPLORATION_RATE = 2.0      # How often to re-check frontiers (seconds)


class AutonomousExplorer(Node):
    """
    Frontier-based autonomous exploration node.

    Finds unexplored areas on the SLAM map and sends Nav2 goals
    to explore them systematically.
    """

    def __init__(self):
        super().__init__('autonomous_explorer')

        # ---- Declare ROS Parameters ----
        self.declare_parameter('min_frontier_size', DEFAULT_MIN_FRONTIER_SIZE)
        self.declare_parameter('max_goal_distance', DEFAULT_MAX_GOAL_DISTANCE)
        self.declare_parameter('min_goal_distance', DEFAULT_MIN_GOAL_DISTANCE)
        self.declare_parameter('goal_timeout', DEFAULT_GOAL_TIMEOUT)
        self.declare_parameter('free_threshold', DEFAULT_FREE_THRESHOLD)
        self.declare_parameter('obstacle_threshold', DEFAULT_OBSTACLE_THRESHOLD)
        self.declare_parameter('exploration_rate', DEFAULT_EXPLORATION_RATE)

        # ---- Read Parameters ----
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.max_goal_distance = self.get_parameter('max_goal_distance').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.free_threshold = self.get_parameter('free_threshold').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.exploration_rate = self.get_parameter('exploration_rate').value

        # ---- State ----
        self.map_data = None            # Latest OccupancyGrid
        self.map_info = None            # Map metadata (resolution, origin)
        self.robot_x = 0.0             # Current robot position (meters)
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.is_navigating = False      # Currently executing a Nav2 goal?
        self.exploration_complete = False
        self.goals_sent = 0
        self.goals_reached = 0
        self.goals_failed = 0
        self.failed_goals = set()       # Track failed goal positions to avoid retrying

        # ---- Callback Group (allows concurrent callbacks) ----
        self.cb_group = ReentrantCallbackGroup()

        # ---- Subscribers ----
        # /map from SLAM Toolbox (transient local for late joiners)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos,
            callback_group=self.cb_group,
        )

        # /odom for robot position
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10,
            callback_group=self.cb_group,
        )

        # ---- Nav2 Action Client ----
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.cb_group,
        )

        # ---- Exploration Timer ----
        self.exploration_timer = self.create_timer(
            self.exploration_rate, self.explore_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info('='*60)
        self.get_logger().info('  AUTONOMOUS EXPLORER — Frontier-Based Exploration')
        self.get_logger().info('='*60)
        self.get_logger().info(f'  Min frontier size : {self.min_frontier_size} cells')
        self.get_logger().info(f'  Max goal distance : {self.max_goal_distance} m')
        self.get_logger().info(f'  Goal timeout      : {self.goal_timeout} s')
        self.get_logger().info('  Waiting for map and Nav2...')
        self.get_logger().info('='*60)

    # =====================================================================
    # CALLBACKS
    # =====================================================================

    def map_callback(self, msg: OccupancyGrid):
        """Store the latest map from SLAM Toolbox."""
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )
        self.map_info = msg.info

    def odom_callback(self, msg: Odometry):
        """Update current robot position from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def explore_callback(self):
        """
        Main exploration loop — called periodically.

        1. Check if we have a map and are not already navigating
        2. Find frontiers on the map
        3. Select the best frontier
        4. Send a Nav2 goal to that frontier
        """
        # Skip if no map yet
        if self.map_data is None:
            self.get_logger().info('Waiting for map data...',
                                    throttle_duration_sec=5.0)
            return

        # Skip if already navigating to a goal
        if self.is_navigating:
            return

        # Skip if exploration is complete
        if self.exploration_complete:
            return

        # Find frontiers
        frontiers = self.find_frontiers()

        if not frontiers:
            self.get_logger().info('='*60)
            self.get_logger().info('  🎉 EXPLORATION COMPLETE!')
            self.get_logger().info(f'  Goals sent    : {self.goals_sent}')
            self.get_logger().info(f'  Goals reached : {self.goals_reached}')
            self.get_logger().info(f'  Goals failed  : {self.goals_failed}')
            self.get_logger().info('  The room has been fully mapped!')
            self.get_logger().info('  Save the map with:')
            self.get_logger().info('    ros2 run nav2_map_server map_saver_cli -f ~/room_map')
            self.get_logger().info('='*60)
            self.exploration_complete = True
            return

        # Select the best frontier
        goal_x, goal_y = self.select_best_frontier(frontiers)

        if goal_x is None:
            self.get_logger().warn('No reachable frontier found, retrying...')
            # Clear failed goals to try again
            if len(self.failed_goals) > 5:
                self.failed_goals.clear()
            return

        # Send navigation goal
        self.send_nav_goal(goal_x, goal_y)

    # =====================================================================
    # FRONTIER DETECTION
    # =====================================================================

    def find_frontiers(self):
        """
        Find all frontier cells on the occupancy grid.

        A frontier cell is a FREE cell that is adjacent (4-connected)
        to at least one UNKNOWN cell.

        Returns a list of frontier clusters, where each cluster is a
        list of (row, col) tuples.
        """
        if self.map_data is None or self.map_info is None:
            return []

        height, width = self.map_data.shape
        frontier_mask = np.zeros((height, width), dtype=bool)

        # Identify free cells
        free_mask = (self.map_data >= 0) & (self.map_data < self.free_threshold)

        # Identify unknown cells
        unknown_mask = (self.map_data == UNKNOWN)

        # For each free cell, check if any 4-connected neighbor is unknown
        # Use array shifting for efficiency instead of per-pixel loops
        # Pad arrays to handle borders
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            # Shift unknown mask in opposite direction
            shifted = np.zeros_like(unknown_mask)
            src_r = slice(max(0, dr), min(height, height + dr) or height)
            src_c = slice(max(0, dc), min(width, width + dc) or width)
            dst_r = slice(max(0, -dr), min(height, height - dr) or height)
            dst_c = slice(max(0, -dc), min(width, width - dc) or width)
            shifted[dst_r, dst_c] = unknown_mask[src_r, src_c]

            frontier_mask |= (free_mask & shifted)

        # Get frontier cell coordinates
        frontier_cells = list(zip(*np.where(frontier_mask)))

        if not frontier_cells:
            return []

        # Cluster frontier cells using BFS (connected components)
        clusters = self._cluster_frontiers(frontier_cells, frontier_mask)

        # Filter out tiny clusters (noise)
        clusters = [c for c in clusters if len(c) >= self.min_frontier_size]

        self.get_logger().info(
            f'Found {len(clusters)} frontier clusters '
            f'({sum(len(c) for c in clusters)} total cells)',
            throttle_duration_sec=5.0,
        )

        return clusters

    def _cluster_frontiers(self, frontier_cells, frontier_mask):
        """
        Group frontier cells into connected clusters using BFS.

        Two frontier cells are in the same cluster if they are
        4-connected (adjacent horizontally or vertically).
        """
        height, width = frontier_mask.shape
        visited = np.zeros_like(frontier_mask)
        clusters = []

        for r, c in frontier_cells:
            if visited[r, c]:
                continue

            # BFS from this cell
            cluster = []
            queue = deque([(r, c)])
            visited[r, c] = True

            while queue:
                cr, cc = queue.popleft()
                cluster.append((cr, cc))

                # Check 4-connected neighbors
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nr, nc = cr + dr, cc + dc
                    if (0 <= nr < height and 0 <= nc < width
                            and not visited[nr, nc]
                            and frontier_mask[nr, nc]):
                        visited[nr, nc] = True
                        queue.append((nr, nc))

            clusters.append(cluster)

        return clusters

    # =====================================================================
    # FRONTIER SELECTION
    # =====================================================================

    def select_best_frontier(self, clusters):
        """
        Select the best frontier cluster to explore.

        Strategy:
          - Convert each cluster to a centroid (world coordinates)
          - Filter out goals too close or too far from the robot
          - Filter out previously failed goals
          - Pick the CLOSEST reachable frontier (greedy nearest-first)

        Nearest-first is simple but effective for room-scale exploration.
        It minimizes travel time and builds a coherent map.
        """
        if not clusters or self.map_info is None:
            return None, None

        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y

        candidates = []

        for cluster in clusters:
            # Compute centroid in world coordinates
            mean_r = sum(r for r, c in cluster) / len(cluster)
            mean_c = sum(c for r, c in cluster) / len(cluster)

            # Convert grid (row, col) → world (x, y)
            world_x = origin_x + mean_c * resolution
            world_y = origin_y + mean_r * resolution

            # Distance from robot
            dist = math.sqrt(
                (world_x - self.robot_x) ** 2 +
                (world_y - self.robot_y) ** 2
            )

            # Filter by distance
            if dist < self.min_goal_distance:
                continue
            if dist > self.max_goal_distance:
                continue

            # Filter out previously failed goals (within 0.3m)
            goal_key = (round(world_x, 1), round(world_y, 1))
            if goal_key in self.failed_goals:
                continue

            # Score: prefer closer frontiers with more cells (bigger unexplored areas)
            # score = cluster_size / distance (bigger & closer = better)
            score = len(cluster) / (dist + 0.1)

            candidates.append((score, world_x, world_y, dist, len(cluster)))

        if not candidates:
            # If no candidates within max distance, try increasing range
            self.max_goal_distance = min(self.max_goal_distance + 0.5, 8.0)
            self.get_logger().info(
                f'Expanding search radius to {self.max_goal_distance:.1f}m'
            )
            return None, None

        # Sort by score (highest first) and pick the best
        candidates.sort(key=lambda x: x[0], reverse=True)
        _, best_x, best_y, best_dist, best_size = candidates[0]

        self.get_logger().info(
            f'Selected frontier: ({best_x:.2f}, {best_y:.2f}), '
            f'dist={best_dist:.2f}m, size={best_size} cells'
        )

        return best_x, best_y

    # =====================================================================
    # NAV2 GOAL SENDING
    # =====================================================================

    def send_nav_goal(self, x, y):
        """
        Send a navigation goal to Nav2 using the NavigateToPose action.

        The robot will:
        1. Plan a path from current position to (x, y)
        2. Follow the path using the DWB controller
        3. Avoid obstacles along the way
        4. Report success or failure
        """
        # Wait for Nav2 action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            return

        # Create goal pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Face toward the goal (compute yaw angle from robot to goal)
        yaw = math.atan2(y - self.robot_y, x - self.robot_x)
        goal_msg.pose.pose.orientation = self._yaw_to_quaternion(yaw)

        self.goals_sent += 1
        self.is_navigating = True

        self.get_logger().info(
            f'[Goal #{self.goals_sent}] Navigating to ({x:.2f}, {y:.2f})'
        )

        # Send goal asynchronously
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback,
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        """Handle Nav2's response to our goal request."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal was REJECTED by Nav2')
            self.is_navigating = False
            self.goals_failed += 1
            return

        self.get_logger().info('Goal accepted, robot is moving...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        """Handle periodic feedback from Nav2 during navigation."""
        feedback = feedback_msg.feedback
        # Could log remaining distance, ETA, etc.
        # Keeping it quiet to avoid log spam
        pass

    def nav_result_callback(self, future):
        """Handle the final result of a Nav2 navigation goal."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.goals_reached += 1
            self.get_logger().info(
                f'✓ Goal reached! ({self.goals_reached}/{self.goals_sent})'
            )
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Goal was canceled')
            self.goals_failed += 1
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Goal was aborted (unreachable?)')
            self.goals_failed += 1
            # Record this position to avoid retrying it
            self.failed_goals.add((
                round(self.robot_x, 1),
                round(self.robot_y, 1),
            ))
        else:
            self.get_logger().warn(f'Goal ended with status: {status}')
            self.goals_failed += 1

        self.is_navigating = False

    # =====================================================================
    # UTILITY FUNCTIONS
    # =====================================================================

    @staticmethod
    def _yaw_to_quaternion(yaw):
        """Convert a yaw angle (radians) to a geometry_msgs Quaternion."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


# ===========================================================================
# MAIN
# ===========================================================================

def main(args=None):
    rclpy.init(args=args)

    explorer = AutonomousExplorer()

    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        explorer.get_logger().info('Exploration stopped by user.')
    finally:
        # Print summary
        explorer.get_logger().info('='*60)
        explorer.get_logger().info('  EXPLORATION SUMMARY')
        explorer.get_logger().info(f'  Goals sent    : {explorer.goals_sent}')
        explorer.get_logger().info(f'  Goals reached : {explorer.goals_reached}')
        explorer.get_logger().info(f'  Goals failed  : {explorer.goals_failed}')
        explorer.get_logger().info('='*60)

        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
