#include <WiFi.h>
#include <HTTPClient.h>
#include <ESP32Servo.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>

// Configuracion 

// Credenciales micro-ROS
char* ssid = "iPhone de Emiliano (2)";
char* password = "Emiliano1";

// Endpoints
char* agent_ip = "172.20.10.4";
int agent_port = 8888;

const char* api_ip = "172.20.10.2";
const int api_port = 5074;
String URL = "http://" + String(api_ip) + ":" + String(api_port) + "/Sensores/color";

// Hardware
Servo servoMotor;
const int SERVO_PIN = 5; 

// Variables Globales
bool cut_completed = false;
int current_H = 0;
int current_S = 0;
int current_V = 0;
String current_plant_name = "Esperando...";

// Timer
unsigned long last_post_time = 0;
const unsigned long POST_INTERVAL = 10000; 

// Objetos de micro-ROS 

rcl_subscription_t h_sub, s_sub, v_sub, name_sub, cut_sub;
std_msgs__msg__Int32 h_msg, s_msg, v_msg;
std_msgs__msg__String plant_name_msg;
std_msgs__msg__Bool cut_msg;

rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_support_t support;
HTTPClient httpClient;
WiFiClient wClient;

// =======================================================
// Funciones
// =======================================================

void performCut() {
  servoMotor.write(0);
  delay(300);
  servoMotor.write(90);
}

void logDataToAPI(String nombre, int h, int s, int v, bool cortada) {
  if (WiFi.status() == WL_CONNECTED) {
    String cortadaStr = cortada ? "true" : "false";
    String postData = "{\"NombrePlanta\":\"" + nombre + "\", \"H\":" + String(h) + ", \"S\":" + String(s) + ", \"V\":" + String(v) + ", \"FueCortada\":" + cortadaStr + "}";

    httpClient.begin(wClient, URL);
    httpClient.addHeader("Content-Type", "application/json");
    httpClient.POST(postData);
    httpClient.end();
  }
}

// =======================================================
// Callbacks ROS
// =======================================================

void h_callback(const void * msgin) {
  const std_msgs__msg__Int32 * m = (const std_msgs__msg__Int32 *)msgin;
  current_H = m->data;
}

void s_callback(const void * msgin) {
  const std_msgs__msg__Int32 * m = (const std_msgs__msg__Int32 *)msgin;
  current_S = m->data;
}

void v_callback(const void * msgin) {
  const std_msgs__msg__Int32 * m = (const std_msgs__msg__Int32 *)msgin;
  current_V = m->data;
}

void plant_name_callback(const void * msgin) {
  const std_msgs__msg__String * m = (const std_msgs__msg__String *)msgin;
  String new_name = String(m->data.data);
  
  if (new_name != current_plant_name) {
    current_plant_name = new_name;
  }
}

void cut_callback(const void * msgin) {
  const std_msgs__msg__Bool * m = (const std_msgs__msg__Bool *)msgin;
  
  if (m->data) {
    performCut();
    logDataToAPI(current_plant_name, current_H, current_S, current_V, true);
  }
}

// =======================================================
// Setup
// =======================================================

void setup() {
  Serial.begin(115200);

  // 1. Hardware Init
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(90);
  delay(500);

  // 2. WiFi Init
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  WiFi.begin(ssid, password);
  
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected. IP: " + WiFi.localIP().toString());

  // 3. micro-ROS Init
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_color_client", "", &support);

  // Memoria para string micro-ROS
  std_msgs__msg__String__init(&plant_name_msg);
  static char memory_buffer[50]; 
  plant_name_msg.data.capacity = 50;
  plant_name_msg.data.data = memory_buffer;
  plant_name_msg.data.size = 0;

  // Subscribers
  rclc_subscription_init_default(&h_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "hsv_h");
  rclc_subscription_init_default(&s_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "hsv_s");
  rclc_subscription_init_default(&v_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "hsv_v");
  rclc_subscription_init_default(&name_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "plant_name");
  rclc_subscription_init_default(&cut_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "cut_plant");

  // Executor
  rclc_executor_init(&executor, &support.context, 5, &allocator);
  rclc_executor_add_subscription(&executor, &h_sub, &h_msg, &h_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &s_sub, &s_msg, &s_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &v_sub, &v_msg, &v_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &name_sub, &plant_name_msg, &plant_name_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &cut_sub, &cut_msg, &cut_callback, ON_NEW_DATA);

  Serial.println("System Ready.");
  last_post_time = millis();
}

// =======================================================
// Main Loop
// =======================================================

void loop() {
  // comunicacion micro-ROS
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  // Monitoreo periodico (10s)
  unsigned long current_time = millis();
  if (current_time - last_post_time >= POST_INTERVAL) {
    last_post_time = current_time;
    logDataToAPI(current_plant_name, current_H, current_S, current_V, false);
  }
  
  delay(10);
}
