#!/usr/bin/env python3
# filepath: /home/emendezj/AgriTech/ros2_ws/src/sensor_logger/sensor_logger/sensor_logger_node.py
"""
AgriTech Sensor Logger Node - Background Service
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge
import cv2
import requests
from datetime import datetime
from enum import Enum
import threading
import math


class TrajectoryState(Enum):
    IDLE = 0
    MOVING = 1
    STOPPING = 2


class SensorLoggerNode(Node):
    def __init__(self):
        super().__init__('sensor_logger_node')

        # ==================== PARAMETERS ====================
        self.declare_parameter('api_host', '172.20.10.2')
        self.declare_parameter('api_port', 5074)
        self.declare_parameter('nombre_planta', 'Planta_1')
        self.declare_parameter('movement_threshold', 0.15)
        self.declare_parameter('stop_timeout', 2.0)
        self.declare_parameter('position_publish_rate', 1.0)
        self.declare_parameter('color_publish_rate', 2.0)

        self.api_host = self.get_parameter('api_host').value
        self.api_port = self.get_parameter('api_port').value
        self.nombre_planta = self.get_parameter('nombre_planta').value
        self.movement_threshold = self.get_parameter('movement_threshold').value
        self.stop_timeout = self.get_parameter('stop_timeout').value

        self.base_url = f"http://{self.api_host}:{self.api_port}"

        # ==================== CV BRIDGE ====================
        self.bridge = CvBridge()

        # ==================== STATE ====================
        self.trajectory_state = TrajectoryState.IDLE
        self.last_movement_time = None
        self.position_x = 0.0
        self.position_y = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.last_imu_time = None
        self.accel_bias_x = 0.0
        self.accel_bias_y = 0.0
        self.bias_alpha = 0.01
        self.last_position_post = None
        self.volume_at_trajectory_start = None
        self.current_volume = 0.0
        self.current_hsv = None
        self.last_color_post = None
        self.lock = threading.Lock()

        # ==================== QoS ====================
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ==================== SUBSCRIBERS ====================
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, qos_best_effort)
        self.volume_sub = self.create_subscription(
            Float32, '/tank/volume', self.volume_callback, 10)
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)

        # ==================== PUBLISHERS ====================
        self.status_pub = self.create_publisher(String, '/agritech/status', 10)

        # ==================== TIMER ====================
        self.state_timer = self.create_timer(0.1, self.state_machine_tick)

        self._log_startup()

    def _log_startup(self):
        self.get_logger().info("=" * 55)
        self.get_logger().info("   AgriTech Sensor Logger")
        self.get_logger().info("=" * 55)
        self.get_logger().info(f"API: {self.base_url}")
        self.get_logger().info(f"Planta: {self.nombre_planta}")
        self.get_logger().info("Waiting for sensor data...")

    # ==================== IMU CALLBACK ====================
    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now()
        now_sec = current_time.nanoseconds / 1e9

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y

        self.accel_bias_x = self.accel_bias_x * (1 - self.bias_alpha) + ax * self.bias_alpha
        self.accel_bias_y = self.accel_bias_y * (1 - self.bias_alpha) + ay * self.bias_alpha

        ax_corrected = ax - self.accel_bias_x
        ay_corrected = ay - self.accel_bias_y

        accel_magnitude = math.sqrt(ax_corrected**2 + ay_corrected**2)
        is_moving = accel_magnitude > self.movement_threshold

        with self.lock:
            if is_moving:
                self.last_movement_time = now_sec
                if self.trajectory_state == TrajectoryState.IDLE:
                    self._start_trajectory()
                if self.trajectory_state == TrajectoryState.STOPPING:
                    self.trajectory_state = TrajectoryState.MOVING

            if self.trajectory_state == TrajectoryState.MOVING:
                self._update_position(ax_corrected, ay_corrected, current_time)

    def _update_position(self, ax, ay, current_time):
        if self.last_imu_time is None:
            self.last_imu_time = current_time
            return

        dt = (current_time - self.last_imu_time).nanoseconds / 1e9
        self.last_imu_time = current_time

        if dt > 0.1 or dt < 0.001:
            return

        if abs(ax) < 0.05:
            ax = 0.0
        if abs(ay) < 0.05:
            ay = 0.0

        self.velocity_x += ax * dt
        self.velocity_y += ay * dt
        self.velocity_x *= 0.98
        self.velocity_y *= 0.98
        self.position_x += self.velocity_x * dt
        self.position_y += self.velocity_y * dt

        now = datetime.now()
        rate = self.get_parameter('position_publish_rate').value
        if self.last_position_post is None or \
           (now - self.last_position_post).total_seconds() >= (1.0 / rate):
            self._post_position()
            self.last_position_post = now

    # ==================== VOLUME CALLBACK ====================
    def volume_callback(self, msg: Float32):
        with self.lock:
            self.current_volume = msg.data
            if self.trajectory_state == TrajectoryState.MOVING and \
               self.volume_at_trajectory_start is None:
                self.volume_at_trajectory_start = msg.data
                self.get_logger().info(f"💧 Tank at start: {msg.data:.1f} ml")

    # ==================== IMAGE CALLBACK ====================
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = cv_image.shape[:2]

            roi_size = 100
            cx, cy = width // 2, height // 2
            x1, x2 = max(0, cx - roi_size), min(width, cx + roi_size)
            y1, y2 = max(0, cy - roi_size), min(height, cy + roi_size)

            roi = cv_image[y1:y2, x1:x2]
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mean_hsv = cv2.mean(hsv_roi)[:3]

            h, s, v = int(mean_hsv[0]), int(mean_hsv[1]), int(mean_hsv[2])
            is_plant_color = (25 <= h <= 95) and (s > 40) and (v > 40)

            if is_plant_color:
                with self.lock:
                    self.current_hsv = {'H': h, 'S': s, 'V': v}
                    now = datetime.now()
                    rate = self.get_parameter('color_publish_rate').value
                    if self.last_color_post is None or \
                       (now - self.last_color_post).total_seconds() >= (1.0 / rate):
                        self._post_color()
                        self.last_color_post = now

        except Exception as e:
            self.get_logger().error(f"Image error: {e}")

    # ==================== STATE MACHINE ====================
    def state_machine_tick(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9

        with self.lock:
            if self.trajectory_state == TrajectoryState.MOVING:
                if self.last_movement_time is not None:
                    if now_sec - self.last_movement_time > self.stop_timeout / 2:
                        self.trajectory_state = TrajectoryState.STOPPING
                        self.get_logger().info("⏸️  Slowing...")

            elif self.trajectory_state == TrajectoryState.STOPPING:
                if self.last_movement_time is not None:
                    if now_sec - self.last_movement_time > self.stop_timeout:
                        self._end_trajectory()

        self._publish_status()

    # ==================== API CALLS ====================
    def _start_trajectory(self):
        self.trajectory_state = TrajectoryState.MOVING
        self.volume_at_trajectory_start = self.current_volume
        self.position_x = 0.0
        self.position_y = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.last_imu_time = None

        try:
            response = requests.post(
                f"{self.base_url}/Sensores/StartTrajectory",
                json={},
                headers={'Content-Type': 'application/json'},
                timeout=2.0
            )
            if response.status_code in [200, 201, 204]:
                self.get_logger().info("🚀 TRAJECTORY STARTED")
        except Exception as e:
            self.get_logger().error(f"StartTrajectory error: {e}")

    def _end_trajectory(self):
        try:
            requests.post(
                f"{self.base_url}/Sensores/CloseTrajectory",
                json={},
                headers={'Content-Type': 'application/json'},
                timeout=2.0
            )
            self.get_logger().info("🏁 TRAJECTORY ENDED")
        except Exception as e:
            self.get_logger().error(f"CloseTrajectory error: {e}")

        self._post_tank_data()
        self.trajectory_state = TrajectoryState.IDLE
        self.volume_at_trajectory_start = None

    def _post_position(self):
        try:
            payload = {
                "X": round(self.position_x * 100, 2),
                "Y": round(self.position_y * 100, 2)
            }
            requests.post(
                f"{self.base_url}/Sensores/PostPosition",
                json=payload,
                headers={'Content-Type': 'application/json'},
                timeout=1.0
            )
            self.get_logger().debug(f"📍 Pos: {payload}")
        except:
            pass

    def _post_color(self):
        if self.current_hsv is None:
            return
        try:
            payload = {
                "NombrePlanta": self.nombre_planta,
                "H": self.current_hsv['H'],
                "S": self.current_hsv['S'],
                "V": self.current_hsv['V'],
                "FueCortada": False
            }
            requests.post(
                f"{self.base_url}/Sensores/color",
                json=payload,
                headers={'Content-Type': 'application/json'},
                timeout=1.0
            )
            self.get_logger().info(f"🌱 Color: {payload}")
        except Exception as e:
            self.get_logger().warn(f"Color error: {e}")

    def _post_tank_data(self):
        if self.volume_at_trajectory_start is None:
            return
        try:
            payload = {
                "ML_Al_Inicio": int(round(self.volume_at_trajectory_start)),
                "ML_Al_Final": int(round(self.current_volume))
            }
            requests.post(
                f"{self.base_url}/Sensores/Tanque/MLPorDia",
                json=payload,
                headers={'Content-Type': 'application/json'},
                timeout=2.0
            )
            used = self.volume_at_trajectory_start - self.current_volume
            self.get_logger().info(f"💧 Tank: {payload['ML_Al_Inicio']} → {payload['ML_Al_Final']} (used: {used:.0f})")
        except Exception as e:
            self.get_logger().error(f"Tank error: {e}")

    def _publish_status(self):
        msg = String()
        msg.data = f"State:{self.trajectory_state.name} Pos:({self.position_x:.2f},{self.position_y:.2f})"
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
