#!/usr/bin/env python3
"""
AgriTech Watering Node v1.1
===========================
Navega a plantas que necesitan agua y activa la bomba.
Espera señal de /agritech/start_watering para iniciar.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Int32, String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import requests
from enum import Enum
import threading
import time


class WateringState(Enum):
    IDLE = 0
    FETCHING_PLANTS = 1
    NAVIGATING = 2
    WATERING = 3
    FINISHED = 4


class WateringNode(Node):
    def __init__(self):
        super().__init__('watering_node')

        # ==================== PARAMETERS ====================
        self.declare_parameter('api_host', '172.20.10.2')
        self.declare_parameter('api_port', 5074)
        self.declare_parameter('humidity_threshold', 50)
        self.declare_parameter('watering_duration', 3.0)
        self.declare_parameter('robot_speed', 0.015)
        self.declare_parameter('plant_detection_cooldown', 2.0)
        self.declare_parameter('min_contour_area', 500)

        # HSV para detectar plantas (verde)
        self.declare_parameter('plant_h_lower', 35)
        self.declare_parameter('plant_h_upper', 85)
        self.declare_parameter('plant_s_lower', 40)
        self.declare_parameter('plant_s_upper', 255)
        self.declare_parameter('plant_v_lower', 40)
        self.declare_parameter('plant_v_upper', 255)

        self.api_host = self.get_parameter('api_host').value
        self.api_port = self.get_parameter('api_port').value
        self.humidity_threshold = self.get_parameter('humidity_threshold').value
        self.watering_duration = self.get_parameter('watering_duration').value
        self.robot_speed = self.get_parameter('robot_speed').value
        self.plant_detection_cooldown = self.get_parameter('plant_detection_cooldown').value
        self.min_contour_area = self.get_parameter('min_contour_area').value

        self.plant_h_lower = self.get_parameter('plant_h_lower').value
        self.plant_h_upper = self.get_parameter('plant_h_upper').value
        self.plant_s_lower = self.get_parameter('plant_s_lower').value
        self.plant_s_upper = self.get_parameter('plant_s_upper').value
        self.plant_v_lower = self.get_parameter('plant_v_lower').value
        self.plant_v_upper = self.get_parameter('plant_v_upper').value

        self.base_url = f"http://{self.api_host}:{self.api_port}"

        # ==================== STATE ====================
        self.state = WateringState.IDLE
        self.plants_to_water = []
        self.current_plant_index = 0
        self.current_plant_id = None
        self.plants_detected = 0
        self.plants_watered = 0
        self.last_detection_time = 0.0
        self.is_watering = False
        self.bridge = CvBridge()
        self.ROI_SIZE = 200

        self.lock = threading.Lock()

        # ==================== QOS ====================
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ==================== SUBSCRIBERS ====================
        self.create_subscription(Bool, '/agritech/start_watering', self.start_callback, 10)
        self.create_subscription(Image, '/video_source/raw', self.image_callback, qos)

        # ==================== PUBLISHERS ====================
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pump_pub = self.create_publisher(Bool, '/actuators/pump', 10)
        self.status_pub = self.create_publisher(String, '/agritech/watering_status', 10)

        # ==================== TIMERS ====================
        self.create_timer(0.1, self.control_loop)
        self.create_timer(2.0, self.publish_status)

        self._log_startup()

    def _log_startup(self):
        self.get_logger().info("=" * 55)
        self.get_logger().info("   💧 AgriTech Watering Node v1.1")
        self.get_logger().info("=" * 55)
        self.get_logger().info(f"API: {self.base_url}")
        self.get_logger().info(f"Humidity threshold: {self.humidity_threshold}%")
        self.get_logger().info(f"Watering duration: {self.watering_duration}s")
        self.get_logger().info("Waiting for /agritech/start_watering...")
        self.get_logger().info("=" * 55)

    # ==================== START CALLBACK ====================

    def start_callback(self, msg: Bool):
        """Espera señal de inicio del mission_controller"""
        if msg.data and self.state == WateringState.IDLE:
            self.get_logger().info("🚀 Watering mission started!")
            self.state = WateringState.FETCHING_PLANTS
            self._reset_state()
            self._fetch_plants_needing_water()

    def _reset_state(self):
        """Reiniciar estado para nueva misión"""
        self.plants_to_water = []
        self.current_plant_index = 0
        self.current_plant_id = None
        self.plants_detected = 0
        self.plants_watered = 0
        self.last_detection_time = 0.0
        self.is_watering = False

    def _fetch_plants_needing_water(self):
        """GET plantas que necesitan agua."""
        try:
            url = f"{self.base_url}/Sensores/get/humedad/necesitan-agua/{self.humidity_threshold}"
            self.get_logger().info(f"📡 Fetching from: {url}")
            
            response = requests.get(url, timeout=5.0)

            if response.status_code == 200:
                data = response.json()

                # Extraer IDs únicos ordenados
                plant_ids = sorted(set(item['iD_Planta'] for item in data))

                if plant_ids:
                    self.plants_to_water = plant_ids
                    self.current_plant_index = 0
                    
                    self.get_logger().info(f"📋 Found {len(plant_ids)} plants needing water: {plant_ids}")
                    self.state = WateringState.NAVIGATING
                    self._set_target_plant()
                else:
                    self.get_logger().info("✅ All plants are well watered!")
                    self._finish_mission()
            else:
                self.get_logger().error(f"API error: {response.status_code}")
                self._finish_mission()

        except Exception as e:
            self.get_logger().error(f"Failed to fetch plants: {e}")
            self._finish_mission()

    def _set_target_plant(self):
        """Establecer la siguiente planta objetivo."""
        if self.current_plant_index < len(self.plants_to_water):
            self.current_plant_id = self.plants_to_water[self.current_plant_index]
            self.get_logger().info(
                f"🎯 Target: Plant {self.current_plant_id} "
                f"({self.current_plant_index + 1}/{len(self.plants_to_water)})"
            )
        else:
            self._finish_mission()

    # ==================== CONTROL LOOP ====================

    def control_loop(self):
        if self.state == WateringState.NAVIGATING and not self.is_watering:
            twist = Twist()
            twist.linear.x = self.robot_speed
            self.cmd_pub.publish(twist)

        elif self.state == WateringState.WATERING:
            self._stop_robot()

        elif self.state in [WateringState.IDLE, WateringState.FINISHED]:
            self._stop_robot()

    def _stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    # ==================== IMAGE PROCESSING ====================

    def image_callback(self, msg: Image):
        if self.state != WateringState.NAVIGATING or self.is_watering:
            return

        current_time = time.time()
        if current_time - self.last_detection_time < self.plant_detection_cooldown:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self._process_image(cv_image, current_time)
        except Exception as e:
            self.get_logger().error(f"Image error: {e}")

    def _process_image(self, cv_image, current_time):
        (h, w) = cv_image.shape[:2]
        half_roi = self.ROI_SIZE // 2
        x1 = max(0, (w // 2) - half_roi)
        y1 = max(0, (h // 2) - half_roi)
        x2 = min(w, (w // 2) + half_roi)
        y2 = min(h, (h // 2) + half_roi)

        roi = cv_image[y1:y2, x1:x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower = np.array([self.plant_h_lower, self.plant_s_lower, self.plant_v_lower])
        upper = np.array([self.plant_h_upper, self.plant_s_upper, self.plant_v_upper])
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > self.min_contour_area:
                self.last_detection_time = current_time
                self.plants_detected += 1

                self.get_logger().info(f"🌱 Detected plant #{self.plants_detected}")

                if self.plants_detected == self.current_plant_id:
                    self.get_logger().info(
                        f"💧 Found target plant {self.current_plant_id}! Watering..."
                    )
                    self._start_watering()

    # ==================== WATERING ====================

    def _start_watering(self):
        """Iniciar riego de la planta actual."""
        self.is_watering = True
        self.state = WateringState.WATERING
        self._stop_robot()

        # Activar bomba
        pump_msg = Bool()
        pump_msg.data = False
        self.pump_pub.publish(pump_msg)
        self.get_logger().info(f"   💦 Pump ON for {self.watering_duration}s...")

        # Programar apagado
        threading.Timer(self.watering_duration, self._stop_watering).start()

    def _stop_watering(self):
        """Detener riego y continuar a la siguiente planta."""
        # Apagar bomba
        pump_msg = Bool()
        pump_msg.data = True
        self.pump_pub.publish(pump_msg)
        self.get_logger().info("   💦 Pump OFF")

        self.plants_watered += 1
        self.is_watering = False

        # Registrar en API
        self._log_watering_to_api()

        # Siguiente planta
        self.current_plant_index += 1
        if self.current_plant_index < len(self.plants_to_water):
            self.state = WateringState.NAVIGATING
            self._set_target_plant()
        else:
            self._finish_mission()

    def _log_watering_to_api(self):
        """Registrar el riego en la API."""
        try:
            payload = {
                "ID_Planta": self.current_plant_id,
                "Humedad_Suelo": 100,
                "Estado_Bomba": True
            }
            requests.post(
                f"{self.base_url}/Sensores/post/humedad",
                json=payload,
                timeout=3.0
            )
            self.get_logger().info(f"   📤 Logged watering for plant {self.current_plant_id}")
        except Exception as e:
            self.get_logger().warn(f"Failed to log watering: {e}")

    # ==================== FINISH ====================

    def _finish_mission(self):
        """Terminar misión de riego."""
        self._stop_robot()

        # Asegurar bomba apagada
        pump_msg = Bool()
        pump_msg.data = False
        self.pump_pub.publish(pump_msg)

        self.get_logger().info("")
        self.get_logger().info("=" * 55)
        self.get_logger().info("   🏁 WATERING MISSION COMPLETE")
        self.get_logger().info("=" * 55)
        self.get_logger().info(f"   📋 Plants needed water: {len(self.plants_to_water)}")
        self.get_logger().info(f"   💧 Plants watered: {self.plants_watered}")
        self.get_logger().info("=" * 55)

        # Publicar que terminó para el mission_controller
        status_msg = String()
        status_msg.data = "COMPLETE"
        self.status_pub.publish(status_msg)

        self.state = WateringState.IDLE

    # ==================== STATUS ====================

    def publish_status(self):
        msg = String()
        if self.state == WateringState.NAVIGATING:
            msg.data = f"[WATER] Target: Plant {self.current_plant_id}, Detected: {self.plants_detected}"
        elif self.state == WateringState.WATERING:
            msg.data = f"[WATER] Watering Plant {self.current_plant_id}"
        elif self.state == WateringState.IDLE:
            msg.data = "[WATER] IDLE - Waiting for start signal"
        else:
            msg.data = f"[WATER] {self.state.name}"

        self.status_pub.publish(msg)

        if self.state not in [WateringState.IDLE, WateringState.FINISHED]:
            self.get_logger().info(msg.data)

    def destroy_node(self):
        self._stop_robot()
        pump_msg = Bool()
        pump_msg.data = False
        self.pump_pub.publish(pump_msg)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WateringNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
