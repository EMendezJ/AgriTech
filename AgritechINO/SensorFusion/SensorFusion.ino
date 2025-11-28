/*
  AgriTech Sensor Fusion - micro-ROS Implementation
  =================================================
  
  ESP32 Node that:
  - Publishes IMU raw data (accelerometer, gyroscope, magnetometer) at 125Hz
  - Publishes ultrasonic distance and tank level at 10Hz
  - Subscribes to actuator commands (servo, pump)
  - Updates LCD locally
  
  Connects to Jetson Nano running ROS2 Humble via micro-ROS Agent
  
  Hardware:
  - ESP32 DevKit
  - LCD I2C 16x2 (0x27)
  - Ultrasonic HC-SR04 (GPIO 16, 17)
  - Servo Motor (GPIO 18)
  - Water Pump Relay (GPIO 19)
  - LSM6DSOX IMU (I2C 0x6A)
  - LIS3MDL Magnetometer (I2C 0x1C)
*/

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

// ROS2 Message Types
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>

// Hardware Libraries
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>

// ==================== CONFIGURATION ====================

// WiFi Configuration - CHANGE THESE!
#define WIFI_SSID "iPhone de Emiliano (2)"
#define WIFI_PASSWORD "Emiliano1"
// micro-ROS Agent Configuration (IP of Jetson Nano)
#define AGENT_IP "172.20.10.2"
#define AGENT_PORT 8888

// Pin Definitions
#define I2C_SDA 21
#define I2C_SCL 22
#define TRIG_PIN 16
#define ECHO_PIN 17
#define SERVO_PIN 18
#define BOMBA_PIN 23  // Changed from GPIO 0 (bootstrap pin) to GPIO 23

// Timing Configuration
#define IMU_PUBLISH_RATE_HZ 125
#define ULTRASONIC_PUBLISH_RATE_HZ 10
#define LCD_UPDATE_RATE_HZ 2

// Tank Dimensions (cm)
#define TANK_WIDTH 7.0f
#define TANK_RECT_HEIGHT 3.5f
#define TANK_RADIUS 3.5f
#define TANK_DEPTH 7.5f
#define TANK_TOTAL_HEIGHT 7.5f

// ==================== MACRO FOR ERROR HANDLING ====================
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// LED for status indication
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// ==================== HARDWARE OBJECTS ====================
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servoMotor;
Adafruit_LSM6DSOX lsm6dsox;
Adafruit_LIS3MDL lis3mdl;

// ==================== MICRO-ROS OBJECTS ====================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

// Publishers
rcl_publisher_t imu_publisher;
rcl_publisher_t mag_publisher;
rcl_publisher_t range_publisher;
rcl_publisher_t tank_level_publisher;
rcl_publisher_t tank_volume_publisher;

// Subscribers
rcl_subscription_t servo_subscriber;
rcl_subscription_t pump_subscriber;

// Timers
rcl_timer_t imu_timer;
rcl_timer_t ultrasonic_timer;

// Messages
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
sensor_msgs__msg__Range range_msg;
std_msgs__msg__Float32 tank_level_msg;
std_msgs__msg__Float32 tank_volume_msg;
std_msgs__msg__Int32 servo_cmd_msg;
std_msgs__msg__Bool pump_cmd_msg;

// ==================== STATE VARIABLES ====================
enum AgentState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

AgentState agent_state = WAITING_AGENT;
bool imu_initialized = false;
bool mag_initialized = false;

float current_distance = 0.0f;
float current_level = 0.0f;
float current_volume = 0.0f;

unsigned long last_lcd_update = 0;
unsigned long last_agent_check = 0;

// Frame ID strings (must persist in memory)
char imu_frame_id[] = "imu_link";
char ultrasonic_frame_id[] = "ultrasonic_link";

// ==================== ERROR HANDLING ====================
void error_loop() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("micro-ROS ERROR");
  lcd.setCursor(0, 1);
  lcd.print("Restarting...");
  
  // Blink LED rapidly for 3 seconds then restart
  for (int i = 0; i < 30; i++) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
  ESP.restart();
}

// ==================== TANK VOLUME CALCULATION ====================
float calculateVolume(float level) {
  if (level <= 0) return 0.0f;
  
  if (level <= TANK_RECT_HEIGHT) {
    // Rectangular section only
    return TANK_WIDTH * level * TANK_DEPTH;
  } else {
    // Rectangular + circular segment
    float volRect = TANK_WIDTH * TANK_RECT_HEIGHT * TANK_DEPTH;
    float h = level - TANK_RECT_HEIGHT;
    if (h > TANK_RADIUS) h = TANK_RADIUS;
    
    float Asegmento = pow(TANK_RADIUS, 2) * acos((TANK_RADIUS - h) / TANK_RADIUS) 
                      - (TANK_RADIUS - h) * sqrt((2 * TANK_RADIUS * h) - h * h);
    float volSegmento = Asegmento * TANK_DEPTH;
    return volRect + volSegmento;
  }
}

// ==================== IMU TIMER CALLBACK (125 Hz) ====================
void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;
  if (!imu_initialized) return;
  
  // Read IMU data
  sensors_event_t accel, gyro, temp;
  lsm6dsox.getEvent(&accel, &gyro, &temp);
  
  // Get current time for header
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  
  // Fill IMU message header
  imu_msg.header.stamp.sec = ts.tv_sec;
  imu_msg.header.stamp.nanosec = ts.tv_nsec;
  
  // Angular velocity (rad/s) - directly from gyroscope
  imu_msg.angular_velocity.x = gyro.gyro.x;
  imu_msg.angular_velocity.y = gyro.gyro.y;
  imu_msg.angular_velocity.z = gyro.gyro.z;
  
  // Linear acceleration (m/s^2) - directly from accelerometer
  imu_msg.linear_acceleration.x = accel.acceleration.x;
  imu_msg.linear_acceleration.y = accel.acceleration.y;
  imu_msg.linear_acceleration.z = accel.acceleration.z;
  
  // Orientation not computed on ESP32 - will be done on Jetson
  // Set covariance[0] to -1 to indicate orientation is not available
  imu_msg.orientation_covariance[0] = -1.0;
  
  // Publish IMU message
  RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  
  // Publish magnetometer separately if available
  if (mag_initialized) {
    sensors_event_t mag;
    lis3mdl.getEvent(&mag);
    
    mag_msg.header.stamp.sec = ts.tv_sec;
    mag_msg.header.stamp.nanosec = ts.tv_nsec;
    
    // Magnetic field in Tesla (convert from Gauss: 1 Gauss = 0.0001 Tesla)
    mag_msg.magnetic_field.x = mag.magnetic.x * 0.0001;
    mag_msg.magnetic_field.y = mag.magnetic.y * 0.0001;
    mag_msg.magnetic_field.z = mag.magnetic.z * 0.0001;
    
    RCSOFTCHECK(rcl_publish(&mag_publisher, &mag_msg, NULL));
  }
}

// ==================== ULTRASONIC TIMER CALLBACK (10 Hz) ====================
void ultrasonic_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;
  
  // Send ultrasonic pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Measure echo duration (timeout 30ms = ~5m max)
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  // Calculate distance and level
  if (duration == 0) {
    current_distance = -1.0f;  // Error indicator
    current_level = 0.0f;
    current_volume = 0.0f;
  } else {
    current_distance = duration * 0.034f / 2.0f;  // Distance in cm
    current_level = TANK_TOTAL_HEIGHT - current_distance;
    if (current_level < 0) current_level = 0;
    if (current_level > TANK_TOTAL_HEIGHT) current_level = TANK_TOTAL_HEIGHT;
    current_volume = calculateVolume(current_level);
  }
  
  // Get timestamp
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  
  // Fill and publish Range message
  range_msg.header.stamp.sec = ts.tv_sec;
  range_msg.header.stamp.nanosec = ts.tv_nsec;
  range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  range_msg.field_of_view = 0.26f;  // ~15 degrees typical for HC-SR04
  range_msg.min_range = 0.02f;       // 2 cm minimum
  range_msg.max_range = 4.0f;        // 400 cm maximum
  range_msg.range = (current_distance > 0) ? (current_distance / 100.0f) : -1.0f;  // Convert to meters
  
  RCSOFTCHECK(rcl_publish(&range_publisher, &range_msg, NULL));
  
  // Publish tank level (in cm)
  tank_level_msg.data = current_level;
  RCSOFTCHECK(rcl_publish(&tank_level_publisher, &tank_level_msg, NULL));
  
  // Publish tank volume (in ml)
  tank_volume_msg.data = current_volume;
  RCSOFTCHECK(rcl_publish(&tank_volume_publisher, &tank_volume_msg, NULL));
}

// ==================== SERVO SUBSCRIPTION CALLBACK ====================
void servo_subscription_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  
  int angle = msg->data;
  
  // Clamp to valid range
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  
  servoMotor.write(angle);
  
  Serial.print("[ROS] SERVO -> ");
  Serial.print(angle);
  Serial.println("°");
}

// ==================== PUMP SUBSCRIPTION CALLBACK ====================
void pump_subscription_callback(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  
  digitalWrite(BOMBA_PIN, msg->data ? HIGH : LOW);
  
  Serial.print("[ROS] PUMP -> ");
  Serial.println(msg->data ? "ON" : "OFF");
}

// ==================== UPDATE LCD ====================
void updateLCD() {
  unsigned long now = millis();
  if (now - last_lcd_update < (1000 / LCD_UPDATE_RATE_HZ)) return;
  last_lcd_update = now;
  
  lcd.clear();
  
  // Line 1: Tank status
  if (current_distance < 0) {
    lcd.setCursor(0, 0);
    lcd.print("Sensor Error!");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("N:");
    lcd.print(current_level, 1);
    lcd.print("cm ");
    lcd.print(current_volume, 0);
    lcd.print("ml");
  }
  
  // Line 2: Connection status
  lcd.setCursor(0, 1);
  switch (agent_state) {
    case WAITING_AGENT:
      lcd.print("Waiting Agent...");
      break;
    case AGENT_AVAILABLE:
      lcd.print("Connecting...");
      break;
    case AGENT_CONNECTED:
      lcd.print("ROS2 Connected");
      break;
    case AGENT_DISCONNECTED:
      lcd.print("Disconnected!");
      break;
  }
}

// ==================== DESTROY MICRO-ROS ENTITIES ====================
void destroyEntities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  
  rcl_publisher_fini(&imu_publisher, &node);
  rcl_publisher_fini(&mag_publisher, &node);
  rcl_publisher_fini(&range_publisher, &node);
  rcl_publisher_fini(&tank_level_publisher, &node);
  rcl_publisher_fini(&tank_volume_publisher, &node);
  rcl_subscription_fini(&servo_subscriber, &node);
  rcl_subscription_fini(&pump_subscriber, &node);
  rcl_timer_fini(&imu_timer);
  rcl_timer_fini(&ultrasonic_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ==================== CREATE MICRO-ROS ENTITIES ====================
bool createEntities() {
  allocator = rcl_get_default_allocator();
  
  // Create init options and support
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) return false;
  
  // Create node
  ret = rclc_node_init_default(&node, "esp32_agritech_node", "", &support);
  if (ret != RCL_RET_OK) return false;
  
  // ===== CREATE PUBLISHERS =====
  
  // IMU Publisher (sensor_msgs/Imu) - Best Effort for high frequency
  ret = rclc_publisher_init_best_effort(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data_raw");
  if (ret != RCL_RET_OK) return false;
  
  // Magnetometer Publisher (sensor_msgs/MagneticField) - Best Effort
  ret = rclc_publisher_init_best_effort(
    &mag_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    "imu/mag");
  if (ret != RCL_RET_OK) return false;
  
  // Range Publisher (sensor_msgs/Range) - Reliable for tank monitoring
  ret = rclc_publisher_init_default(
    &range_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "ultrasonic/range");
  if (ret != RCL_RET_OK) return false;
  
  // Tank Level Publisher (std_msgs/Float32)
  ret = rclc_publisher_init_default(
    &tank_level_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "tank/level");
  if (ret != RCL_RET_OK) return false;
  
  // Tank Volume Publisher (std_msgs/Float32)
  ret = rclc_publisher_init_default(
    &tank_volume_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "tank/volume");
  if (ret != RCL_RET_OK) return false;
  
  // ===== CREATE SUBSCRIBERS =====
  
  // Servo Subscriber (std_msgs/Int32)
  ret = rclc_subscription_init_default(
    &servo_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "actuators/servo");
  if (ret != RCL_RET_OK) return false;
  
  // Pump Subscriber (std_msgs/Bool)
  ret = rclc_subscription_init_default(
    &pump_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "actuators/pump");
  if (ret != RCL_RET_OK) return false;
  
  // ===== CREATE TIMERS =====
  
  // IMU Timer (125 Hz = 8ms period)
  const unsigned int imu_timer_period_ms = 1000 / IMU_PUBLISH_RATE_HZ;
  ret = rclc_timer_init_default(
    &imu_timer,
    &support,
    RCL_MS_TO_NS(imu_timer_period_ms),
    imu_timer_callback);
  if (ret != RCL_RET_OK) return false;
  
  // Ultrasonic Timer (10 Hz = 100ms period)
  const unsigned int ultrasonic_timer_period_ms = 1000 / ULTRASONIC_PUBLISH_RATE_HZ;
  ret = rclc_timer_init_default(
    &ultrasonic_timer,
    &support,
    RCL_MS_TO_NS(ultrasonic_timer_period_ms),
    ultrasonic_timer_callback);
  if (ret != RCL_RET_OK) return false;
  
  // ===== CREATE EXECUTOR =====
  // 2 timers + 2 subscribers = 4 handles
  ret = rclc_executor_init(&executor, &support.context, 4, &allocator);
  if (ret != RCL_RET_OK) return false;
  
  ret = rclc_executor_add_timer(&executor, &imu_timer);
  if (ret != RCL_RET_OK) return false;
  
  ret = rclc_executor_add_timer(&executor, &ultrasonic_timer);
  if (ret != RCL_RET_OK) return false;
  
  ret = rclc_executor_add_subscription(
    &executor, 
    &servo_subscriber, 
    &servo_cmd_msg, 
    &servo_subscription_callback, 
    ON_NEW_DATA);
  if (ret != RCL_RET_OK) return false;
  
  ret = rclc_executor_add_subscription(
    &executor, 
    &pump_subscriber, 
    &pump_cmd_msg,
    &pump_subscription_callback, 
    ON_NEW_DATA);
  if (ret != RCL_RET_OK) return false;
  
  // ===== INITIALIZE MESSAGE FRAME IDS =====
  imu_msg.header.frame_id.data = imu_frame_id;
  imu_msg.header.frame_id.size = strlen(imu_frame_id);
  imu_msg.header.frame_id.capacity = sizeof(imu_frame_id);
  
  mag_msg.header.frame_id.data = imu_frame_id;
  mag_msg.header.frame_id.size = strlen(imu_frame_id);
  mag_msg.header.frame_id.capacity = sizeof(imu_frame_id);
  
  range_msg.header.frame_id.data = ultrasonic_frame_id;
  range_msg.header.frame_id.size = strlen(ultrasonic_frame_id);
  range_msg.header.frame_id.capacity = sizeof(ultrasonic_frame_id);
  
  // Sync time with agent
  rmw_uros_sync_session(1000);
  
  return true;
}

// ==================== INITIALIZE HARDWARE ====================
void initializeHardware() {
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);  // 400kHz I2C
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("AgriTech v2.0");
  lcd.setCursor(0, 1);
  lcd.print("micro-ROS");
  
  // Initialize pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BOMBA_PIN, OUTPUT);
  digitalWrite(BOMBA_PIN, HIGH);  // Ensure pump is OFF
  
  // Initialize Servo
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(0);  // Neutral position
  
  delay(500);
  
  // Initialize IMU (LSM6DSOX)
  Serial.print("Initializing LSM6DSOX... ");
  if (lsm6dsox.begin_I2C(0x6A, &Wire)) {
    lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    lsm6dsox.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm6dsox.setGyroDataRate(LSM6DS_RATE_104_HZ);
    imu_initialized = true;
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
    lcd.setCursor(0, 1);
    lcd.print("IMU Error!");
  }
  
  // Initialize Magnetometer (LIS3MDL)
  Serial.print("Initializing LIS3MDL... ");
  if (lis3mdl.begin_I2C(0x1C, &Wire)) {
    lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
    mag_initialized = true;
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
  }
  
  delay(500);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n\n========================================");
  Serial.println("   AgriTech micro-ROS Node v2.0");
  Serial.println("========================================\n");
  
  // Initialize all hardware
  initializeHardware();
  
  // Configure micro-ROS WiFi transport
  Serial.println("Configuring WiFi transport...");
  Serial.print("  SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("  Agent: ");
  Serial.print(AGENT_IP);
  Serial.print(":");
  Serial.println(AGENT_PORT);
  
  set_microros_wifi_transports(
    (char*)WIFI_SSID, 
    (char*)WIFI_PASSWORD, 
    (char*)AGENT_IP, 
    AGENT_PORT
  );
  
  lcd.clear();
  lcd.print("Connecting...");
  
  delay(2000);
  
  agent_state = WAITING_AGENT;
  
  Serial.println("\nSetup complete. Waiting for micro-ROS agent...");
  Serial.println("Start agent with: ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888\n");
}

// ==================== LOOP ====================
void loop() {
  unsigned long now = millis();
  
  // State machine for agent connection
  switch (agent_state) {
    
    case WAITING_AGENT:
      // Check for agent every 500ms
      if (now - last_agent_check > 500) {
        last_agent_check = now;
        if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
          agent_state = AGENT_AVAILABLE;
          Serial.println("[micro-ROS] Agent detected!");
        }
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // Blink while waiting
      }
      break;
      
    case AGENT_AVAILABLE:
      Serial.println("[micro-ROS] Creating entities...");
      if (createEntities()) {
        agent_state = AGENT_CONNECTED;
        Serial.println("[micro-ROS] Connected successfully!");
        digitalWrite(LED_BUILTIN, HIGH);  // LED on when connected
      } else {
        Serial.println("[micro-ROS] Failed to create entities, retrying...");
        agent_state = WAITING_AGENT;
        destroyEntities();
      }
      break;
      
    case AGENT_CONNECTED:
      // Check agent connection periodically
      if (now - last_agent_check > 200) {
        last_agent_check = now;
        if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
          Serial.println("[micro-ROS] Agent disconnected!");
          agent_state = AGENT_DISCONNECTED;
          digitalWrite(LED_BUILTIN, LOW);
        }
      }
      
      // Spin executor (handles timers and subscriptions)
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      break;
      
    case AGENT_DISCONNECTED:
      Serial.println("[micro-ROS] Cleaning up and reconnecting...");
      destroyEntities();
      agent_state = WAITING_AGENT;
      break;
  }
  
  // Always update LCD (local display independent of ROS)
  updateLCD();
}