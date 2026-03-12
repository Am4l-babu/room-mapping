/**
 * ===========================================================================
 * main.cpp — ESP32 micro-ROS Firmware for Differential Drive Mapping Robot
 * ===========================================================================
 *
 * This is the MAIN FIRMWARE that runs on the real ESP32 robot.
 * It replaces all Gazebo simulation plugins with real hardware drivers.
 *
 * ARCHITECTURE:
 *
 *   ┌─────────────────────────────────────────────────────────┐
 *   │                    ESP32 (this firmware)                 │
 *   │                                                         │
 *   │   /cmd_vel subscriber ──→ Motor Control (L293D)         │
 *   │   /odom publisher    ←── Wheel Encoders / Open-loop     │
 *   │   /imu publisher     ←── MPU6050 (I2C)                  │
 *   │   /scan publisher    ←── Ultrasonic + Servo sweep       │
 *   │                                                         │
 *   │   micro-ROS Agent ←──WiFi UDP──→ PC (ROS2 network)     │
 *   └─────────────────────────────────────────────────────────┘
 *
 * SIMULATION ↔ REAL ROBOT MAPPING:
 *
 *   Gazebo Plugin              →  ESP32 Hardware
 *   ─────────────────────────────────────────────
 *   gazebo_ros_diff_drive      →  L293D + DC motors + encoders
 *   gazebo_ros_ray_sensor      →  HC-SR04 + SG90 servo sweep
 *   gazebo_ros_imu_sensor      →  MPU6050 (I2C)
 *   Gazebo physics             →  Real world
 *
 *   SAME ROS2 TOPICS:
 *   /cmd_vel  (input)   — Velocity commands from teleop or nav2
 *   /odom     (output)  — Odometry for SLAM and navigation
 *   /imu      (output)  — IMU data for orientation
 *   /scan     (output)  — Laser scan for SLAM mapping
 *
 *   Because the topics are identical, SLAM Toolbox, RViz, and all
 *   other ROS2 nodes work WITHOUT ANY CHANGES between sim and real!
 *
 * SETUP STEPS:
 *   1. Edit include/config.h with your WiFi and agent IP
 *   2. Build: pio run
 *   3. Upload: pio run --target upload
 *   4. Start agent on PC: ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
 *   5. Power on ESP32 — it auto-connects to the agent
 *   6. Run SLAM: ros2 launch room_mapping_robot slam_launch.py use_sim_time:=false
 *
 * ===========================================================================
 */

#include <Arduino.h>
#include <Wire.h>

// micro-ROS headers
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS2 message types
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>

// Robot configuration
#include "config.h"


// ===========================================================================
// micro-ROS Infrastructure
// ===========================================================================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

// Subscriber: /cmd_vel (velocity commands from teleop or navigation)
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

// Publisher: /odom (wheel odometry)
rcl_publisher_t odom_pub;
nav_msgs__msg__Odometry odom_msg;

// Publisher: /imu (MPU6050 data)
rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;

// Publisher: /scan (ultrasonic sweep as LaserScan)
rcl_publisher_t scan_pub;
sensor_msgs__msg__LaserScan scan_msg;

// Timer for periodic publishing
rcl_timer_t odom_timer;
rcl_timer_t imu_timer;


// ===========================================================================
// Robot State Variables
// ===========================================================================

// Current commanded velocity (from /cmd_vel)
volatile float target_linear_vel = 0.0;
volatile float target_angular_vel = 0.0;

// Odometry state (accumulated position)
float odom_x = 0.0;
float odom_y = 0.0;
float odom_theta = 0.0;
float odom_linear_vel = 0.0;
float odom_angular_vel = 0.0;

// IMU raw data
float accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
float gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;

// Ultrasonic scan data
#define NUM_SCAN_POINTS ((SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) / SERVO_STEP + 1)
float scan_ranges[NUM_SCAN_POINTS];
int current_servo_angle = SERVO_MIN_ANGLE;
bool sweep_direction = true;  // true = forward, false = backward

// Timing
unsigned long last_odom_time = 0;
unsigned long last_scan_time = 0;
unsigned long last_blink_time = 0;
bool led_state = false;
bool agent_connected = false;


// ===========================================================================
// ERROR HANDLING MACROS
// ===========================================================================
// These macros check micro-ROS return codes and handle errors gracefully

#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
        Serial.printf("micro-ROS error: %s at line %d\n", \
                      rcl_get_error_string().str, __LINE__); \
        rcl_reset_error(); \
        return; \
    } \
}

#define RCSOFTCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
        Serial.printf("micro-ROS soft error at line %d\n", __LINE__); \
    } \
}


// ===========================================================================
// MOTOR CONTROL
// ===========================================================================

/**
 * Initialize L293D motor driver pins and PWM channels.
 *
 * The L293D has two H-bridges, each controlled by:
 *   - IN1/IN2 (or IN3/IN4): direction control
 *   - EN (enable): speed control via PWM
 *
 * Direction truth table (per motor):
 *   IN1=HIGH, IN2=LOW  → Forward
 *   IN1=LOW,  IN2=HIGH → Reverse
 *   IN1=LOW,  IN2=LOW  → Coast (free spin)
 *   IN1=HIGH, IN2=HIGH → Brake
 */
void setupMotors() {
    // Direction pins
    pinMode(MOTOR_LEFT_IN1, OUTPUT);
    pinMode(MOTOR_LEFT_IN2, OUTPUT);
    pinMode(MOTOR_RIGHT_IN3, OUTPUT);
    pinMode(MOTOR_RIGHT_IN4, OUTPUT);

    // PWM speed control using ESP32 LEDC peripheral
    ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_LEFT_EN, PWM_CHANNEL_LEFT);

    ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_RIGHT_EN, PWM_CHANNEL_RIGHT);

    // Start with motors stopped
    stopMotors();

    Serial.println("[MOTORS] L293D motor driver initialized");
}

/**
 * Stop both motors immediately.
 */
void stopMotors() {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    digitalWrite(MOTOR_RIGHT_IN3, LOW);
    digitalWrite(MOTOR_RIGHT_IN4, LOW);
    ledcWrite(PWM_CHANNEL_LEFT, 0);
    ledcWrite(PWM_CHANNEL_RIGHT, 0);
}

/**
 * Set individual motor speed and direction.
 *
 * @param channel  PWM channel (left or right)
 * @param in1_pin  Forward direction pin
 * @param in2_pin  Reverse direction pin
 * @param speed    Speed value (-255 to +255). Positive = forward.
 */
void setMotor(int channel, int in1_pin, int in2_pin, int speed) {
    if (speed > 0) {
        // Forward
        digitalWrite(in1_pin, HIGH);
        digitalWrite(in2_pin, LOW);
        ledcWrite(channel, constrain(speed, MIN_PWM, MAX_PWM));
    } else if (speed < 0) {
        // Reverse
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, HIGH);
        ledcWrite(channel, constrain(-speed, MIN_PWM, MAX_PWM));
    } else {
        // Stop
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, LOW);
        ledcWrite(channel, 0);
    }
}

/**
 * Convert cmd_vel (linear.x, angular.z) to individual wheel speeds.
 *
 * Differential drive kinematics:
 *   v_left  = linear_vel - (angular_vel * wheel_separation / 2)
 *   v_right = linear_vel + (angular_vel * wheel_separation / 2)
 *
 * Then convert wheel velocity (m/s) to PWM (0-255):
 *   pwm = (v_wheel / max_wheel_vel) * MAX_PWM
 */
void driveMotors(float linear_vel, float angular_vel) {
    // Differential drive inverse kinematics
    float v_left = linear_vel - (angular_vel * WHEEL_SEPARATION / 2.0f);
    float v_right = linear_vel + (angular_vel * WHEEL_SEPARATION / 2.0f);

    // Maximum wheel velocity (m/s) at full PWM
    float max_wheel_vel = MAX_LINEAR_VEL + (MAX_ANGULAR_VEL * WHEEL_SEPARATION / 2.0f);

    // Convert to PWM values
    int pwm_left = (int)((v_left / max_wheel_vel) * MAX_PWM);
    int pwm_right = (int)((v_right / max_wheel_vel) * MAX_PWM);

    // Apply to motors
    setMotor(PWM_CHANNEL_LEFT, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, pwm_left);
    setMotor(PWM_CHANNEL_RIGHT, MOTOR_RIGHT_IN3, MOTOR_RIGHT_IN4, pwm_right);

    // Store for odometry estimation
    odom_linear_vel = linear_vel;
    odom_angular_vel = angular_vel;
}


// ===========================================================================
// MPU6050 IMU
// ===========================================================================

/**
 * Initialize MPU6050 over I2C.
 *
 * The MPU6050 provides:
 *   - 3-axis accelerometer (±2g, ±4g, ±8g, ±16g)
 *   - 3-axis gyroscope (±250, ±500, ±1000, ±2000 °/s)
 *   - Digital temperature sensor
 *
 * We use it in default mode (±2g accel, ±250°/s gyro).
 */
void setupIMU() {
    Wire.begin(IMU_SDA, IMU_SCL);

    // Wake up MPU6050 (default is sleep mode)
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0x00);  // Clear sleep bit
    Wire.endTransmission(true);

    // Configure accelerometer (±2g range)
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x1C);  // ACCEL_CONFIG register
    Wire.write(0x00);  // ±2g
    Wire.endTransmission(true);

    // Configure gyroscope (±250°/s range)
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x1B);  // GYRO_CONFIG register
    Wire.write(0x00);  // ±250°/s
    Wire.endTransmission(true);

    Serial.println("[IMU] MPU6050 initialized (I2C)");
}

/**
 * Read raw data from MPU6050 and convert to SI units.
 *
 * Accelerometer: raw / 16384.0 = g's → multiply by 9.81 for m/s²
 * Gyroscope: raw / 131.0 = °/s → multiply by (π/180) for rad/s
 */
void readIMU() {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x3B);  // Starting register (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDRESS, (uint8_t)14, (uint8_t)true);

    // Read 14 bytes: accel (6) + temp (2) + gyro (6)
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    int16_t temp_raw = (Wire.read() << 8) | Wire.read();  // Temperature (unused)
    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();

    (void)temp_raw;  // Suppress unused warning

    // Convert to SI units
    accel_x = (float)ax / 16384.0f * 9.81f;  // m/s²
    accel_y = (float)ay / 16384.0f * 9.81f;
    accel_z = (float)az / 16384.0f * 9.81f;
    gyro_x = (float)gx / 131.0f * (PI / 180.0f);  // rad/s
    gyro_y = (float)gy / 131.0f * (PI / 180.0f);
    gyro_z = (float)gz / 131.0f * (PI / 180.0f);
}


// ===========================================================================
// ULTRASONIC SENSOR + SERVO SWEEP
// ===========================================================================

/**
 * Initialize the ultrasonic sensor and sweep servo.
 *
 * The HC-SR04 measures distance by:
 *   1. Send 10μs HIGH pulse on TRIG pin
 *   2. Measure duration of HIGH pulse on ECHO pin
 *   3. Distance = (duration × speed_of_sound) / 2
 *
 * The servo sweeps the sensor across 0°-180° to create
 * a distance scan similar to a LiDAR.
 */
void setupUltrasonic() {
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);

    // Initialize servo on the specified pin
    // Note: Using ledcSetup for servo (50Hz PWM)
    ledcSetup(2, 50, 16);  // Channel 2, 50Hz, 16-bit
    ledcAttachPin(SERVO_PIN, 2);

    // Move to starting position
    setServoAngle(SERVO_MIN_ANGLE);
    delay(500);

    Serial.println("[ULTRASONIC] HC-SR04 + servo initialized");
}

/**
 * Set the servo angle (0-180 degrees).
 *
 * Converts angle to PWM duty cycle:
 *   0°   → ~500μs  pulse (duty ~1638 at 16-bit 50Hz)
 *   180° → ~2500μs pulse (duty ~8192 at 16-bit 50Hz)
 */
void setServoAngle(int angle) {
    int duty = map(angle, 0, 180, 1638, 8192);
    ledcWrite(2, duty);
}

/**
 * Measure distance using HC-SR04 ultrasonic sensor.
 *
 * @return Distance in meters (0 if out of range)
 */
float measureDistance() {
    // Send trigger pulse
    digitalWrite(ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);

    // Measure echo duration
    unsigned long duration = pulseIn(ULTRASONIC_ECHO, HIGH, ULTRASONIC_TIMEOUT);

    if (duration == 0) {
        return 0.0f;  // Timeout — no echo received
    }

    // Convert to meters: duration(μs) × speed_of_sound(m/μs) / 2
    // Speed of sound ≈ 0.000343 m/μs at 20°C
    float distance_m = (float)duration * 0.000343f / 2.0f;

    // Clamp to max range
    if (distance_m > (float)ULTRASONIC_MAX_CM / 100.0f) {
        return 0.0f;
    }

    return distance_m;
}

/**
 * Perform one step of the ultrasonic sweep.
 *
 * This function is called periodically to:
 * 1. Take a distance reading at the current angle
 * 2. Store it in the scan_ranges array
 * 3. Move the servo to the next angle
 * 4. When a full sweep is complete, publish the /scan message
 */
void updateScan() {
    // Calculate array index from current angle
    int index = (current_servo_angle - SERVO_MIN_ANGLE) / SERVO_STEP;

    // Take distance measurement
    scan_ranges[index] = measureDistance();

    // Move to next angle
    if (sweep_direction) {
        current_servo_angle += SERVO_STEP;
        if (current_servo_angle > SERVO_MAX_ANGLE) {
            current_servo_angle = SERVO_MAX_ANGLE;
            sweep_direction = false;
            // Full sweep complete — scan is ready to publish
            publishScan();
        }
    } else {
        current_servo_angle -= SERVO_STEP;
        if (current_servo_angle < SERVO_MIN_ANGLE) {
            current_servo_angle = SERVO_MIN_ANGLE;
            sweep_direction = true;
            // Full sweep complete — scan is ready to publish
            publishScan();
        }
    }

    // Move servo to new angle
    setServoAngle(current_servo_angle);
}


// ===========================================================================
// IR OBSTACLE SENSORS
// ===========================================================================

/**
 * Initialize IR obstacle detection sensors.
 */
void setupIRSensors() {
    pinMode(IR_SENSOR_LEFT, INPUT);
    pinMode(IR_SENSOR_RIGHT, INPUT);
    Serial.println("[IR] Obstacle sensors initialized");
}

/**
 * Check if any obstacle is detected by IR sensors.
 *
 * @return true if obstacle detected (emergency stop needed)
 */
bool checkObstacles() {
    bool left_obstacle = digitalRead(IR_SENSOR_LEFT) == (IR_ACTIVE_LOW ? LOW : HIGH);
    bool right_obstacle = digitalRead(IR_SENSOR_RIGHT) == (IR_ACTIVE_LOW ? LOW : HIGH);
    return left_obstacle || right_obstacle;
}


// ===========================================================================
// ODOMETRY
// ===========================================================================

/**
 * Update odometry estimate using dead reckoning.
 *
 * Without encoders (USE_ENCODERS=false):
 *   We estimate position from commanded velocities.
 *   This drifts over time but SLAM corrects it.
 *
 * With encoders (USE_ENCODERS=true):
 *   Position is calculated from actual wheel rotations.
 *   Much more accurate.
 *
 * Odometry equations (differential drive):
 *   Δx = linear_vel × cos(θ) × Δt
 *   Δy = linear_vel × sin(θ) × Δt
 *   Δθ = angular_vel × Δt
 */
void updateOdometry() {
    unsigned long now = millis();
    float dt = (now - last_odom_time) / 1000.0f;  // Convert to seconds
    last_odom_time = now;

    if (dt > 0.5f || dt <= 0.0f) {
        return;  // Skip if time delta is unreasonable
    }

    // Update position using current velocities
    float delta_x = odom_linear_vel * cos(odom_theta) * dt;
    float delta_y = odom_linear_vel * sin(odom_theta) * dt;
    float delta_theta = odom_angular_vel * dt;

    odom_x += delta_x;
    odom_y += delta_y;
    odom_theta += delta_theta;

    // Normalize theta to [-π, π]
    while (odom_theta > PI) odom_theta -= 2.0f * PI;
    while (odom_theta < -PI) odom_theta += 2.0f * PI;
}


// ===========================================================================
// micro-ROS CALLBACKS
// ===========================================================================

/**
 * Callback for /cmd_vel messages.
 *
 * This is called whenever a Twist message arrives on /cmd_vel.
 * In the real robot, this replaces what gazebo_ros_diff_drive does in sim.
 *
 * The same teleop_twist_keyboard or nav2 commands that work in
 * Gazebo will work here — no changes needed!
 */
void cmdVelCallback(const void *msg_in) {
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msg_in;

    target_linear_vel = msg->linear.x;
    target_angular_vel = msg->angular.z;

    // Safety: check IR obstacle sensors
    if (checkObstacles() && target_linear_vel > 0) {
        // Obstacle ahead — stop forward motion but allow turning/reversing
        Serial.println("[SAFETY] Obstacle detected! Blocking forward motion.");
        target_linear_vel = 0.0;
    }

    // Apply to motors
    driveMotors(target_linear_vel, target_angular_vel);
}

/**
 * Timer callback: publish odometry at fixed rate.
 */
void odomTimerCallback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) return;

    updateOdometry();

    // Fill odometry message
    // Header
    odom_msg.header.stamp.sec = (int32_t)(millis() / 1000);
    odom_msg.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);
    // frame_id is set during setup

    // Position
    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.position.z = 0.0;

    // Orientation (quaternion from yaw angle)
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(odom_theta / 2.0f);
    odom_msg.pose.pose.orientation.w = cos(odom_theta / 2.0f);

    // Velocity
    odom_msg.twist.twist.linear.x = odom_linear_vel;
    odom_msg.twist.twist.angular.z = odom_angular_vel;

    RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));
}

/**
 * Timer callback: publish IMU data at fixed rate.
 */
void imuTimerCallback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) return;

    readIMU();

    // Fill IMU message
    imu_msg.header.stamp.sec = (int32_t)(millis() / 1000);
    imu_msg.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);

    // Angular velocity (from gyroscope)
    imu_msg.angular_velocity.x = gyro_x;
    imu_msg.angular_velocity.y = gyro_y;
    imu_msg.angular_velocity.z = gyro_z;

    // Linear acceleration (from accelerometer)
    imu_msg.linear_acceleration.x = accel_x;
    imu_msg.linear_acceleration.y = accel_y;
    imu_msg.linear_acceleration.z = accel_z;

    // Orientation (from odometry theta — for basic orientation)
    // A better approach would use a complementary or Kalman filter
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = sin(odom_theta / 2.0f);
    imu_msg.orientation.w = cos(odom_theta / 2.0f);

    RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
}


// ===========================================================================
// SCAN PUBLISHING
// ===========================================================================

/**
 * Publish a complete ultrasonic sweep as a LaserScan message.
 *
 * The ultrasonic sweep covers 0°-180° (π radians) with SERVO_STEP increments.
 * This is published as a sensor_msgs/LaserScan, which is the same format
 * as the simulated LiDAR in Gazebo.
 *
 * SLAM Toolbox processes both identically!
 */
void publishScan() {
    scan_msg.header.stamp.sec = (int32_t)(millis() / 1000);
    scan_msg.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);

    // Angle range: 0° to 180° converted to radians
    scan_msg.angle_min = 0.0f;
    scan_msg.angle_max = PI;
    scan_msg.angle_increment = (float)SERVO_STEP * (PI / 180.0f);
    scan_msg.time_increment = (float)SCAN_UPDATE_MS / 1000.0f;
    scan_msg.scan_time = (float)(NUM_SCAN_POINTS * SCAN_UPDATE_MS) / 1000.0f;
    scan_msg.range_min = 0.02f;  // 2cm minimum
    scan_msg.range_max = (float)ULTRASONIC_MAX_CM / 100.0f;

    // Copy scan data
    scan_msg.ranges.size = NUM_SCAN_POINTS;
    for (int i = 0; i < NUM_SCAN_POINTS; i++) {
        scan_msg.ranges.data[i] = scan_ranges[i];
    }

    RCSOFTCHECK(rcl_publish(&scan_pub, &scan_msg, NULL));
}


// ===========================================================================
// micro-ROS SETUP
// ===========================================================================

/**
 * Initialize micro-ROS: connect to agent, create node, publishers,
 * subscribers, and timers.
 */
bool setupMicroROS() {
    Serial.println("[micro-ROS] Connecting to agent...");

    // Configure WiFi transport
    set_microros_wifi_transports(
        (char*)WIFI_SSID,
        (char*)WIFI_PASSWORD,
        (char*)AGENT_IP,
        AGENT_PORT
    );

    delay(2000);  // Wait for WiFi connection

    // Initialize micro-ROS allocator
    allocator = rcl_get_default_allocator();

    // Create micro-ROS support structure
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        Serial.println("[micro-ROS] Failed to connect to agent!");
        return false;
    }

    Serial.println("[micro-ROS] Connected to agent!");

    // Create node: "esp32_robot_node"
    RCCHECK(rclc_node_init_default(
        &node, "esp32_robot_node", "", &support));

    // ---- Create /cmd_vel subscriber ----
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        TOPIC_CMD_VEL));

    // ---- Create /odom publisher ----
    RCCHECK(rclc_publisher_init_default(
        &odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        TOPIC_ODOM));

    // ---- Create /imu publisher ----
    RCCHECK(rclc_publisher_init_default(
        &imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        TOPIC_IMU));

    // ---- Create /scan publisher ----
    RCCHECK(rclc_publisher_init_default(
        &scan_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        TOPIC_SCAN));

    // ---- Create odometry timer (20 Hz) ----
    RCCHECK(rclc_timer_init_default(
        &odom_timer, &support,
        RCL_MS_TO_NS(ODOM_PUBLISH_MS),
        odomTimerCallback));

    // ---- Create IMU timer (100 Hz) ----
    RCCHECK(rclc_timer_init_default(
        &imu_timer, &support,
        RCL_MS_TO_NS(IMU_PUBLISH_MS),
        imuTimerCallback));

    // ---- Create executor (handles callbacks) ----
    // 3 handles: 1 subscriber + 2 timers
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &cmd_vel_sub, &cmd_vel_msg,
        &cmdVelCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));

    // ---- Initialize message frame IDs ----
    // These static strings are used in message headers
    // They must match the URDF frame names!
    odom_msg.header.frame_id.data = (char*)FRAME_ODOM;
    odom_msg.header.frame_id.size = strlen(FRAME_ODOM);
    odom_msg.header.frame_id.capacity = strlen(FRAME_ODOM) + 1;
    odom_msg.child_frame_id.data = (char*)FRAME_BASE;
    odom_msg.child_frame_id.size = strlen(FRAME_BASE);
    odom_msg.child_frame_id.capacity = strlen(FRAME_BASE) + 1;

    imu_msg.header.frame_id.data = (char*)FRAME_IMU;
    imu_msg.header.frame_id.size = strlen(FRAME_IMU);
    imu_msg.header.frame_id.capacity = strlen(FRAME_IMU) + 1;

    scan_msg.header.frame_id.data = (char*)FRAME_LIDAR;
    scan_msg.header.frame_id.size = strlen(FRAME_LIDAR);
    scan_msg.header.frame_id.capacity = strlen(FRAME_LIDAR) + 1;

    // Allocate scan ranges array
    scan_msg.ranges.data = (float*)malloc(NUM_SCAN_POINTS * sizeof(float));
    scan_msg.ranges.size = NUM_SCAN_POINTS;
    scan_msg.ranges.capacity = NUM_SCAN_POINTS;

    Serial.println("[micro-ROS] Node, publishers, subscribers ready!");
    return true;
}


// ===========================================================================
// ARDUINO SETUP
// ===========================================================================

void setup() {
    // ---- Serial for debugging ----
    Serial.begin(115200);
    delay(1000);

    Serial.println("=========================================");
    Serial.println("  Room Mapping Robot — ESP32 Firmware");
    Serial.println("  ROS2 Jazzy + micro-ROS");
    Serial.println("=========================================");

    // ---- Status LED ----
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // ---- Initialize hardware ----
    setupMotors();
    setupIMU();
    setupUltrasonic();
    setupIRSensors();

    // ---- Initialize micro-ROS ----
    agent_connected = setupMicroROS();

    if (agent_connected) {
        Serial.println("[STARTUP] All systems GO! Robot is ready.");
        digitalWrite(LED_PIN, HIGH);
    } else {
        Serial.println("[STARTUP] WARNING: micro-ROS agent not connected.");
        Serial.println("[STARTUP] Robot will retry connection...");
    }

    last_odom_time = millis();
    last_scan_time = millis();
}


// ===========================================================================
// ARDUINO MAIN LOOP
// ===========================================================================

void loop() {
    if (agent_connected) {
        // ---- Process micro-ROS callbacks ----
        // This handles incoming /cmd_vel and fires timer callbacks
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        // ---- Update ultrasonic sweep ----
        unsigned long now = millis();
        if (now - last_scan_time >= SCAN_UPDATE_MS) {
            last_scan_time = now;
            updateScan();
        }

        // ---- Status LED: steady blink when connected ----
        if (now - last_blink_time >= LED_BLINK_CONNECTED) {
            last_blink_time = now;
            led_state = !led_state;
            digitalWrite(LED_PIN, led_state);
        }

    } else {
        // ---- Not connected: try to reconnect ----
        unsigned long now = millis();

        // Fast blink to indicate searching
        if (now - last_blink_time >= LED_BLINK_SEARCHING) {
            last_blink_time = now;
            led_state = !led_state;
            digitalWrite(LED_PIN, led_state);
        }

        // Retry connection every 5 seconds
        static unsigned long last_retry = 0;
        if (now - last_retry >= 5000) {
            last_retry = now;
            Serial.println("[micro-ROS] Retrying agent connection...");
            agent_connected = setupMicroROS();
        }

        // Stop motors when disconnected (safety)
        stopMotors();
    }

    // ---- Watchdog: stop motors if no cmd_vel received ----
    // If the connection drops, the robot should stop
    // This is handled by the micro-ROS timeout mechanism
}
