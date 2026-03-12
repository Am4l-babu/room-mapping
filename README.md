# Room Mapping Robot — ROS2 Jazzy Simulation + ESP32 Firmware

## Complete Differential Drive SLAM Mapping Robot

This project provides a **complete simulation** of a differential drive room-mapping robot
using ROS2 Jazzy, Gazebo, and SLAM Toolbox — plus **real ESP32 firmware** for the physical robot.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        SIMULATION (Gazebo)                          │
│                                                                     │
│   Gazebo World (room.world)                                        │
│   ├── gazebo_ros_diff_drive  → /odom, odom→base_footprint TF      │
│   ├── gazebo_ros_ray_sensor  → /scan (LaserScan)                   │
│   └── gazebo_ros_imu_sensor  → /imu (Imu)                          │
│                              ← /cmd_vel (Twist)                     │
└─────────────────────────────────────────────────────────────────────┘
                                  │
                    SAME ROS2 TOPICS (drop-in replacement)
                                  │
┌─────────────────────────────────────────────────────────────────────┐
│                        REAL ROBOT (ESP32)                            │
│                                                                     │
│   ESP32 + micro-ROS (WiFi UDP)                                     │
│   ├── L293D + DC motors      → /odom (from encoders/open-loop)     │
│   ├── HC-SR04 + Servo sweep  → /scan (LaserScan from ultrasonic)   │
│   └── MPU6050 (I2C)          → /imu (Imu)                          │
│                              ← /cmd_vel (Twist)                     │
└─────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      ROS2 SLAM STACK (PC)                           │
│                                                                     │
│   robot_state_publisher    → TF: base_link → sensors               │
│   SLAM Toolbox             → /map + TF: map → odom                 │
│   RViz2                    → Visualization                          │
│   teleop_twist_keyboard    → /cmd_vel                               │
└─────────────────────────────────────────────────────────────────────┘
```

---

## TF Tree

```
map                          ← Published by SLAM Toolbox
└── odom                     ← Published by diff_drive plugin / ESP32
    └── base_footprint       ← Ground-level reference
        └── base_link        ← Robot chassis
            ├── left_wheel_link
            ├── right_wheel_link
            ├── caster_wheel
            ├── lidar_link   ← 2D LiDAR (sim) / Ultrasonic (real)
            └── imu_link     ← IMU sensor (MPU6050)
```

---

## Project Structure

```
ros2_room_mapping_ws/
├── src/
│   └── room_mapping_robot/
│       ├── package.xml              # ROS2 package manifest
│       ├── CMakeLists.txt           # Build configuration
│       ├── launch/
│       │   ├── gazebo_launch.py     # Main: Gazebo + robot + state publisher
│       │   ├── spawn_robot.launch.py # Standalone robot spawner
│       │   └── slam_launch.py       # SLAM Toolbox + RViz
│       ├── urdf/
│       │   └── robot.urdf.xacro     # Complete robot model with plugins
│       ├── config/
│       │   └── slam_params.yaml     # SLAM Toolbox parameters
│       ├── worlds/
│       │   └── room.world           # Gazebo indoor room environment
│       ├── rviz/
│       │   └── mapping.rviz         # RViz layout for mapping
│       └── scripts/
│           └── teleop_helper.py     # Keyboard control helper
├── esp32_firmware/
│   ├── platformio.ini               # PlatformIO build config
│   ├── include/
│   │   └── config.h                 # Pin definitions & WiFi config
│   └── src/
│       └── main.cpp                 # ESP32 micro-ROS firmware
└── README.md                        # This file
```

---

## Prerequisites

### System Requirements
- **Ubuntu 24.04** (Noble Numbat)
- **ROS2 Jazzy** (installed and sourced)
- **Gazebo Classic** (gazebo11)

### Install ROS2 Dependencies

```bash
# Update package list
sudo apt update

# Install Gazebo Classic and ROS2 integration
sudo apt install -y \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-gazebo-ros \
    ros-jazzy-gazebo-plugins

# Install SLAM Toolbox
sudo apt install -y ros-jazzy-slam-toolbox

# Install Navigation2 map server (for saving maps)
sudo apt install -y ros-jazzy-nav2-map-server

# Install robot model tools
sudo apt install -y \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher

# Install teleoperation
sudo apt install -y ros-jazzy-teleop-twist-keyboard

# Install RViz2
sudo apt install -y ros-jazzy-rviz2

# Install TF2 tools (for debugging TF tree)
sudo apt install -y ros-jazzy-tf2-tools
```

### For ESP32 Firmware (Optional — only for real robot)

```bash
# Install PlatformIO
pip install platformio

# Or install PlatformIO IDE extension in VS Code
```

---

## Step-by-Step: Running the Simulation

### Step 1: Build the Workspace

```bash
# Navigate to workspace
cd ~/ros2_room_mapping_ws

# Build with colcon
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

> **Tip:** Add `source ~/ros2_room_mapping_ws/install/setup.bash` to your `~/.bashrc`

### Step 2: Launch Gazebo with the Robot

```bash
# Terminal 1: Launch Gazebo + robot + state publisher
ros2 launch room_mapping_robot gazebo_launch.py
```

**What happens:**
- Gazebo opens showing a 5×5m room with obstacles
- The robot (blue box with wheels) appears at position (1.0, 1.0)
- Robot state publisher starts broadcasting TF frames
- LiDAR beams become visible in Gazebo (green lines)

**Wait until** you see the robot in Gazebo before proceeding.

### Step 3: Start SLAM Toolbox + RViz

```bash
# Terminal 2: Launch SLAM and RViz
ros2 launch room_mapping_robot slam_launch.py
```

**What happens:**
- SLAM Toolbox starts processing /scan data
- RViz opens with the preconfigured mapping view
- You should see:
  - Robot model (blue box with wheels)
  - Red dots = LiDAR scan points
  - Grey/white grid = map being built
  - TF frames as colored axes

### Step 4: Control the Robot with Keyboard

```bash
# Terminal 3: Start keyboard teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Or use the helper script:**
```bash
ros2 run room_mapping_robot teleop_helper.py
```

**Keyboard controls:**
```
   u    i    o        Forward (left/straight/right)
   j    k    l        Turn (left/stop/right)
   m    ,    .        Backward (left/straight/right)

   q/z  = increase/decrease all speeds
   w/x  = increase/decrease linear speed
   e/c  = increase/decrease angular speed
   k    = emergency stop
```

**Mapping strategy:**
1. Drive slowly along walls (builds wall outlines)
2. Make a complete loop around the room perimeter
3. Drive through the center, passing near obstacles
4. Return to start (closes the loop → improves accuracy)

### Step 5: Save the Map

When you're satisfied with the map:

```bash
# Terminal 4: Save map to file
ros2 run nav2_map_server map_saver_cli -f ~/room_map
```

This creates:
- `~/room_map.pgm` — Grayscale image of the map
- `~/room_map.yaml` — Map metadata (resolution, origin, etc.)

---

## Useful Debugging Commands

```bash
# View all active topics
ros2 topic list

# Check /scan data is flowing
ros2 topic hz /scan

# Check /odom data is flowing
ros2 topic hz /odom

# View the TF tree (generates a PDF)
ros2 run tf2_tools view_frames

# Echo a specific topic
ros2 topic echo /odom --once

# Check robot description
ros2 topic echo /robot_description --once

# List all active nodes
ros2 node list

# View node info
ros2 node info /slam_toolbox
```

---

## Connecting the Real ESP32 Robot

### Hardware Setup

| Component | Connection |
|-----------|-----------|
| L293D IN1 | ESP32 GPIO 25 |
| L293D IN2 | ESP32 GPIO 26 |
| L293D IN3 | ESP32 GPIO 27 |
| L293D IN4 | ESP32 GPIO 14 |
| L293D EN1 | ESP32 GPIO 32 (PWM) |
| L293D EN2 | ESP32 GPIO 33 (PWM) |
| MPU6050 SDA | ESP32 GPIO 21 |
| MPU6050 SCL | ESP32 GPIO 22 |
| HC-SR04 TRIG | ESP32 GPIO 18 |
| HC-SR04 ECHO | ESP32 GPIO 19 |
| Servo Signal | ESP32 GPIO 13 |
| IR Left | ESP32 GPIO 34 |
| IR Right | ESP32 GPIO 35 |

### Firmware Upload

```bash
# 1. Edit WiFi and agent IP
#    Open esp32_firmware/include/config.h
#    Set WIFI_SSID, WIFI_PASSWORD, AGENT_IP

# 2. Build firmware
cd esp32_firmware
pio run

# 3. Upload to ESP32
pio run --target upload

# 4. Monitor serial output
pio device monitor
```

### Running with Real Robot

```bash
# Terminal 1: Start micro-ROS agent (bridges ESP32 ↔ ROS2)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# Terminal 2: Start robot state publisher (NO Gazebo needed!)
ros2 launch room_mapping_robot spawn_robot.launch.py

# Terminal 3: Start SLAM (note: use_sim_time:=false for real robot!)
ros2 launch room_mapping_robot slam_launch.py use_sim_time:=false

# Terminal 4: Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Key Differences: Simulation vs Real Robot

| Aspect | Simulation | Real Robot |
|--------|-----------|------------|
| use_sim_time | true | **false** |
| LiDAR source | gazebo_ros_ray_sensor | HC-SR04 + servo sweep |
| Motor control | gazebo_ros_diff_drive | ESP32 + L293D |
| IMU source | gazebo_ros_imu_sensor | MPU6050 via I2C |
| Odometry | Perfect (from plugin) | Estimated (encoders/open-loop) |
| Communication | Internal ROS2 | WiFi UDP via micro-ROS |
| World | Gazebo room.world | Real room |

### Why This Architecture Works

The **critical insight** is that both simulation and real robot publish the **exact same ROS2 topics**:

- `/cmd_vel` — Velocity commands (input)
- `/odom` — Odometry (output)
- `/scan` — Laser scan (output)
- `/imu` — IMU data (output)
- `/tf` — Transform tree (output)

SLAM Toolbox, RViz, and teleop don't know or care whether the data comes from Gazebo or real hardware. **The interface is the topic, not the implementation.**

---

## Troubleshooting

### Gazebo won't start
```bash
# Kill any existing Gazebo processes
killall -9 gazebo gzserver gzclient
# Retry launch
```

### No map appearing in RViz
1. Check Fixed Frame is set to `map` in RViz
2. Verify SLAM is receiving scans: `ros2 topic hz /scan`
3. Verify TF is complete: `ros2 run tf2_tools view_frames`
4. Drive the robot — the map only builds when the robot moves!

### Robot doesn't move
1. Check `/cmd_vel` is being published: `ros2 topic echo /cmd_vel`
2. Make sure the teleop terminal has focus (it reads keyboard input)
3. Check Gazebo is not paused (bottom bar shows play/pause)

### TF errors in terminal
- "Could not find a connection between 'map' and 'odom'"
  → SLAM Toolbox hasn't started yet. Wait for it to process the first scan.
- "Lookup would require extrapolation into the future"
  → Clock mismatch. Make sure `use_sim_time: true` is set everywhere.

---

## License

MIT License — Free to use, modify, and distribute.
