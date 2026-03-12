#!/usr/bin/env python3
"""
============================================================================
teleop_helper.py — Teleoperation Helper Script
============================================================================

This script provides a convenient wrapper around teleop_twist_keyboard
with pre-configured settings for our small differential drive robot.

WHY THIS EXISTS:
  teleop_twist_keyboard defaults to high speeds that would be unrealistic
  for our tiny robot (0.18m chassis). This helper:
  1. Prints clear instructions for the user
  2. Launches teleop_twist_keyboard with appropriate speed limits
  3. Shows the current robot status

USAGE:
  # Option 1: Run this helper
  $ ros2 run room_mapping_robot teleop_helper.py
  
  # Option 2: Run teleop_twist_keyboard directly with custom speeds
  $ ros2 run teleop_twist_keyboard teleop_twist_keyboard \
      --ros-args -p speed:=0.15 -p turn:=0.5

KEYBOARD CONTROLS (from teleop_twist_keyboard):
  Moving around:
     u    i    o        (forward-left, forward, forward-right)
     j    k    l        (turn-left, stop, turn-right)
     m    ,    .        (backward-left, backward, backward-right)

  Speed control:
     q/z  : increase/decrease max speeds by 10%
     w/x  : increase/decrease only linear speed by 10%
     e/c  : increase/decrease only angular speed by 10%

  Stop: k or spacebar

HOW THIS CONNECTS TO THE ROBOT:
  Simulation: teleop → /cmd_vel → gazebo_ros_diff_drive → wheel joints
  Real robot: teleop → /cmd_vel → micro-ROS → ESP32 → L293D → DC motors
  
  The /cmd_vel topic is the universal interface. Same keyboard commands
  work in both simulation and real robot!

MAPPING TIPS:
  1. Drive slowly along walls to build accurate wall outlines
  2. Make complete loops around the room for loop closure
  3. Approach obstacles from multiple angles
  4. Avoid fast spinning (confuses scan matching)
  5. Return to starting position to close the loop
============================================================================
"""

import subprocess
import sys
import os


def print_banner():
    """Print a helpful banner with instructions."""
    banner = """
╔══════════════════════════════════════════════════════════════════╗
║           ROOM MAPPING ROBOT — TELEOPERATION CONTROL           ║
╠══════════════════════════════════════════════════════════════════╣
║                                                                  ║
║  This launches keyboard control for the mapping robot.           ║
║                                                                  ║
║  KEYBOARD CONTROLS:                                              ║
║     u    i    o     ← Forward (left/straight/right)              ║
║     j    k    l     ← Turn (left/stop/right)                     ║
║     m    ,    .     ← Backward (left/straight/right)             ║
║                                                                  ║
║  SPEED:                                                          ║
║     q/z = all speeds ↑/↓    w/x = linear ↑/↓                    ║
║     e/c = angular ↑/↓       k = STOP                             ║
║                                                                  ║
║  MAPPING TIPS:                                                   ║
║     • Drive slowly along walls                                   ║
║     • Make full loops around the room                            ║
║     • Approach obstacles from multiple angles                    ║
║     • Avoid fast spinning                                        ║
║                                                                  ║
║  Robot specs: wheel_radius=0.03m, separation=0.16m               ║
║  Recommended: linear=0.15 m/s, angular=0.5 rad/s                ║
║                                                                  ║
║  Press Ctrl+C to exit teleoperation                              ║
║                                                                  ║
╚══════════════════════════════════════════════════════════════════╝
"""
    print(banner)


def main():
    """Launch teleop_twist_keyboard with robot-appropriate settings."""
    print_banner()

    # Build the command to run teleop_twist_keyboard
    # We set initial speed and turn rate appropriate for our small robot
    cmd = [
        'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
        '--ros-args',
        '-p', 'speed:=0.15',      # 0.15 m/s linear (appropriate for small robot)
        '-p', 'turn:=0.5',         # 0.5 rad/s angular
        '-p', 'repeat_rate:=10.0', # Repeat rate for held keys
    ]

    print(f"\n[INFO] Starting teleop_twist_keyboard...")
    print(f"[INFO] Publishing to: /cmd_vel")
    print(f"[INFO] Initial speed: 0.15 m/s linear, 0.5 rad/s angular")
    print(f"[INFO] Press Ctrl+C to stop\n")

    try:
        # Run teleop_twist_keyboard as a subprocess
        # stdin is passed through so keyboard input works
        process = subprocess.run(cmd, check=True)
    except KeyboardInterrupt:
        print("\n\n[INFO] Teleoperation stopped by user.")
    except FileNotFoundError:
        print("\n[ERROR] teleop_twist_keyboard not found!")
        print("[ERROR] Install it with:")
        print("  sudo apt install ros-jazzy-teleop-twist-keyboard")
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        print(f"\n[ERROR] teleop_twist_keyboard exited with error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
