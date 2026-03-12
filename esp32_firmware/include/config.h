/**
 * ===========================================================================
 * config.h — Hardware Pin Definitions and Configuration
 * ===========================================================================
 *
 * This file defines ALL hardware connections between the ESP32 and the
 * robot's sensors, actuators, and peripherals.
 *
 * WIRING DIAGRAM:
 *
 *   ESP32 Pin  →  Component         →  Function
 *   ─────────────────────────────────────────────────
 *   GPIO 25    →  L293D IN1         →  Left motor forward
 *   GPIO 26    →  L293D IN2         →  Left motor reverse
 *   GPIO 27    →  L293D IN3         →  Right motor forward
 *   GPIO 14    →  L293D IN4         →  Right motor reverse
 *   GPIO 32    →  L293D EN1 (PWM)   →  Left motor speed
 *   GPIO 33    →  L293D EN2 (PWM)   →  Right motor speed
 *   GPIO 21    →  MPU6050 SDA       →  I2C data (IMU)
 *   GPIO 22    →  MPU6050 SCL       →  I2C clock (IMU)
 *   GPIO 18    →  HC-SR04 TRIG      →  Ultrasonic trigger
 *   GPIO 19    →  HC-SR04 ECHO      →  Ultrasonic echo
 *   GPIO 13    →  SG90 Servo        →  Ultrasonic sweep
 *   GPIO 34    →  IR Sensor Left    →  Obstacle detection (input only)
 *   GPIO 35    →  IR Sensor Right   →  Obstacle detection (input only)
 *   GPIO  5    →  SD Card CS        →  SPI chip select
 *   GPIO 23    →  SD Card MOSI      →  SPI data out
 *   GPIO 19    →  SD Card MISO      →  SPI data in (shared with ECHO)
 *   GPIO 18    →  SD Card SCK       →  SPI clock (shared with TRIG)
 *   GPIO 16    →  Left Encoder A    →  Wheel encoder (optional)
 *   GPIO 17    →  Right Encoder A   →  Wheel encoder (optional)
 *
 * NOTE: GPIO 34, 35 are input-only pins on ESP32 (used for IR sensors)
 * NOTE: Some pins are shared — adjust if using SD card simultaneously
 *
 * ===========================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

// ===========================================================================
// WiFi Configuration (for micro-ROS WiFi transport)
// ===========================================================================
// Change these to match your WiFi network
#define WIFI_SSID          "YourWiFiSSID"       // Your WiFi network name
#define WIFI_PASSWORD      "YourWiFiPassword"    // Your WiFi password

// ===========================================================================
// micro-ROS Agent Configuration
// ===========================================================================
// The micro-ROS agent runs on your PC and bridges ESP32 ↔ ROS2
// Start the agent with:
//   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
#define AGENT_IP           "192.168.1.100"      // IP of PC running the agent
#define AGENT_PORT         8888                  // UDP port (must match agent)

// ===========================================================================
// Motor Driver Pins (L293D)
// ===========================================================================
// The L293D is a dual H-bridge motor driver
// IN1/IN2 control direction of left motor
// IN3/IN4 control direction of right motor
// EN1/EN2 control speed via PWM

// Left Motor
#define MOTOR_LEFT_IN1     25    // GPIO 25 → L293D IN1 (forward)
#define MOTOR_LEFT_IN2     26    // GPIO 26 → L293D IN2 (reverse)
#define MOTOR_LEFT_EN      32    // GPIO 32 → L293D EN1 (PWM speed)

// Right Motor
#define MOTOR_RIGHT_IN3    27    // GPIO 27 → L293D IN3 (forward)
#define MOTOR_RIGHT_IN4    14    // GPIO 14 → L293D IN4 (reverse)
#define MOTOR_RIGHT_EN     33    // GPIO 33 → L293D EN2 (PWM speed)

// PWM Configuration for motor speed control
#define PWM_FREQUENCY      5000  // 5 kHz PWM frequency
#define PWM_RESOLUTION     8     // 8-bit resolution (0-255)
#define PWM_CHANNEL_LEFT   0     // LEDC channel for left motor
#define PWM_CHANNEL_RIGHT  1     // LEDC channel for right motor
#define MAX_PWM            255   // Maximum PWM value (full speed)
#define MIN_PWM            60    // Minimum PWM to overcome motor inertia

// ===========================================================================
// Robot Physical Parameters
// ===========================================================================
// These MUST match the URDF model for consistent behavior
#define WHEEL_RADIUS       0.03f   // 0.03 meters (30mm)
#define WHEEL_SEPARATION   0.16f   // 0.16 meters (160mm)
#define MAX_LINEAR_VEL     0.3f    // Maximum linear velocity (m/s)
#define MAX_ANGULAR_VEL    1.5f    // Maximum angular velocity (rad/s)

// ===========================================================================
// Wheel Encoder Pins (Optional — for closed-loop odometry)
// ===========================================================================
// If your DC motors have encoders, connect them here
// If no encoders, odometry will be estimated from PWM commands (open-loop)
#define USE_ENCODERS       false   // Set to true if encoders are installed
#define ENCODER_LEFT_A     16      // GPIO 16 → Left encoder channel A
#define ENCODER_LEFT_B     4       // GPIO  4 → Left encoder channel B
#define ENCODER_RIGHT_A    17      // GPIO 17 → Right encoder channel A
#define ENCODER_RIGHT_B    2       // GPIO  2 → Right encoder channel B
#define ENCODER_TICKS_REV  20      // Encoder ticks per wheel revolution

// ===========================================================================
// IMU (MPU6050) Configuration
// ===========================================================================
// The MPU6050 is a 6-axis IMU (3-axis accelerometer + 3-axis gyroscope)
// Connected via I2C bus
#define IMU_SDA            21      // GPIO 21 → MPU6050 SDA
#define IMU_SCL            22      // GPIO 22 → MPU6050 SCL
#define MPU6050_ADDRESS    0x68    // I2C address (0x68 or 0x69)
#define IMU_UPDATE_RATE_MS 10      // 100 Hz update rate (10ms period)

// ===========================================================================
// Ultrasonic Sensor (HC-SR04) + Servo (SG90) Configuration
// ===========================================================================
// The ultrasonic sensor is mounted on a servo that sweeps 180°
// to create a distance scan (poor man's LiDAR)
#define ULTRASONIC_TRIG    18      // GPIO 18 → HC-SR04 TRIG pin
#define ULTRASONIC_ECHO    19      // GPIO 19 → HC-SR04 ECHO pin
#define SERVO_PIN          13      // GPIO 13 → SG90 servo signal
#define SERVO_MIN_ANGLE    0       // Minimum sweep angle (degrees)
#define SERVO_MAX_ANGLE    180     // Maximum sweep angle (degrees)
#define SERVO_STEP         5       // Angle step per reading (degrees)
#define ULTRASONIC_MAX_CM  400     // Maximum range (cm)
#define ULTRASONIC_TIMEOUT 30000   // Echo timeout (microseconds)
#define SCAN_UPDATE_MS     100     // Time between sweep steps (ms)

// ===========================================================================
// IR Obstacle Sensors Configuration
// ===========================================================================
// Digital IR proximity sensors for emergency stop
// Output LOW when obstacle is detected (active low)
#define IR_SENSOR_LEFT     34      // GPIO 34 → Left IR sensor (input only)
#define IR_SENSOR_RIGHT    35      // GPIO 35 → Right IR sensor (input only)
#define IR_ACTIVE_LOW      true    // true = LOW means obstacle detected

// ===========================================================================
// SD Card Module (SPI) Configuration
// ===========================================================================
// Used for local data logging (backup to ROS2 bag recording)
#define SD_CS              5       // GPIO  5 → SD card CS (chip select)
#define SD_MOSI            23      // GPIO 23 → SD card MOSI
#define SD_MISO            19      // GPIO 19 → SD card MISO (shared!)
#define SD_SCK             18      // GPIO 18 → SD card SCK  (shared!)
#define ENABLE_SD_LOGGING  false   // Set to true to enable SD logging

// ===========================================================================
// ROS2 Topic Names
// ===========================================================================
// These must match the topics used in the ROS2 simulation
// so the same SLAM / RViz configuration works for both
#define TOPIC_CMD_VEL      "/cmd_vel"     // Incoming velocity commands
#define TOPIC_ODOM         "/odom"        // Outgoing odometry
#define TOPIC_IMU          "/imu"         // Outgoing IMU data
#define TOPIC_SCAN         "/scan"        // Outgoing laser scan
#define FRAME_ODOM         "odom"         // Odometry frame ID
#define FRAME_BASE         "base_footprint"  // Robot base frame ID
#define FRAME_IMU          "imu_link"     // IMU frame ID
#define FRAME_LIDAR        "lidar_link"   // LiDAR/ultrasonic frame ID

// ===========================================================================
// Timing Configuration
// ===========================================================================
#define ODOM_PUBLISH_MS    50      // Odometry publish interval (20 Hz)
#define IMU_PUBLISH_MS     10      // IMU publish interval (100 Hz)
#define SCAN_PUBLISH_MS    100     // Scan publish interval (10 Hz)
#define AGENT_TIMEOUT_MS   2000    // micro-ROS agent connection timeout

// ===========================================================================
// Status LED
// ===========================================================================
#define LED_PIN            2       // Built-in LED on most ESP32 boards
#define LED_BLINK_CONNECTED  1000  // Blink interval when connected (ms)
#define LED_BLINK_SEARCHING  200   // Blink interval when searching (ms)

#endif // CONFIG_H
