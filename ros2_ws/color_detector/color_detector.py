import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import Twist
import time


class ColorDetectorROS2(Node):
    def __init__(self):
        super().__init__("color_detector")

        # ==================== PARAMETERS ====================
        self.declare_parameter('h_lower', 0)
        self.declare_parameter('s_lower', 100)
        self.declare_parameter('v_lower', 100)
        self.declare_parameter('h_upper', 10)
        self.declare_parameter('s_upper', 255)
        self.declare_parameter('v_upper', 255)
        self.declare_parameter('min_area', 500)
        self.declare_parameter('plant_name', 'Tomate')
        self.declare_parameter('total_plants', 4)
        self.declare_parameter('robot_speed', 0.01)
        self.declare_parameter('backup_speed', -0.005)
        self.declare_parameter('detection_cooldown', 3.0)
        self.declare_parameter('servo_cut_angle', 150)
        self.declare_parameter('servo_rest_angle', 0)
        self.declare_parameter('cut_duration', 1.0)

        self.H_LOWER = self.get_parameter('h_lower').value
        self.S_LOWER = self.get_parameter('s_lower').value
        self.V_LOWER = self.get_parameter('v_lower').value
        self.H_UPPER = self.get_parameter('h_upper').value
        self.S_UPPER = self.get_parameter('s_upper').value
        self.V_UPPER = self.get_parameter('v_upper').value
        self.MIN_CONTOUR_AREA = self.get_parameter('min_area').value
        self.plant_name_base = self.get_parameter('plant_name').value
        self.TOTAL_PLANTS = self.get_parameter('total_plants').value
        self.ROBOT_SPEED = self.get_parameter('robot_speed').value
        self.BACKUP_SPEED = self.get_parameter('backup_speed').value
        self.DETECTION_COOLDOWN = self.get_parameter('detection_cooldown').value
        self.SERVO_CUT_ANGLE = self.get_parameter('servo_cut_angle').value
        self.SERVO_REST_ANGLE = self.get_parameter('servo_rest_angle').value
        self.CUT_DURATION = self.get_parameter('cut_duration').value

        # ==================== STATE ====================
        self.plants_checked = 0
        self.plants_cut = 0
        self.recorrido_activo = False  # ← INACTIVO hasta recibir señal
        self.is_cutting = False
        self.last_detection_time = 0.0

        self.ROI_SIZE = 200
        self.frame_count = 0
        self.bridge = CvBridge()

        # ==================== SUBSCRIBERS ====================
        self.create_subscription(Image, "/video_source/raw", self.image_callback, 10)
        self.create_subscription(Bool, "/agritech/start_harvest", self.start_callback, 10)

        # ==================== PUBLISHERS ====================
        self.h_pub = self.create_publisher(Int32, "/hsv_h", 10)
        self.s_pub = self.create_publisher(Int32, "/hsv_s", 10)
        self.v_pub = self.create_publisher(Int32, "/hsv_v", 10)
        self.name_pub = self.create_publisher(String, "/plant_name", 10)
        self.check_pub = self.create_publisher(Bool, "/plant_checked", 10)
        self.cut_pub = self.create_publisher(Bool, "/cut_plant", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.servo_pub = self.create_publisher(Int32, "/servo_angle", 10)
        self.status_pub = self.create_publisher(String, "/agritech/harvest_status", 10)

        # ==================== TIMERS ====================
        self.movement_timer = self.create_timer(0.1, self.control_loop)
        self.create_timer(2.0, self.publish_status)

        self._log_startup()

    def _log_startup(self):
        self.get_logger().info("=" * 55)
        self.get_logger().info("   🌱 AgriTech Color Detector v3.2")
        self.get_logger().info("=" * 55)
        self.get_logger().info(f"Plantas a revisar: {self.TOTAL_PLANTS}")
        self.get_logger().info(f"HSV Rango: H({self.H_LOWER}-{self.H_UPPER}) "
                               f"S({self.S_LOWER}-{self.S_UPPER}) V({self.V_LOWER}-{self.V_UPPER})")
        self.get_logger().info(f"Servo: corte={self.SERVO_CUT_ANGLE}° reposo={self.SERVO_REST_ANGLE}°")
        self.get_logger().info("Waiting for /agritech/start_harvest...")
        self.get_logger().info("=" * 55)

    # ==================== START CALLBACK ====================

    def start_callback(self, msg: Bool):
        """Espera señal de inicio del mission_controller"""
        if msg.data and not self.recorrido_activo:
            self.get_logger().info("🚀 Harvest started!")
            self.recorrido_activo = True
            self.plants_checked = 0
            self.plants_cut = 0
            self.last_detection_time = 0.0

    # ==================== CONTROL ====================

    def control_loop(self):
        if not self.recorrido_activo or self.is_cutting:
            return
        self.publish_velocity(self.ROBOT_SPEED)

    def publish_velocity(self, speed):
        twist = Twist()
        twist.linear.x = speed
        self.cmd_pub.publish(twist)

    def publish_servo(self, angle):
        msg = Int32()
        msg.data = angle
        self.servo_pub.publish(msg)

    # ==================== CUT SEQUENCE ====================

    def execute_cut_sequence(self):
        self.is_cutting = True

        self.get_logger().info("   ↩️ Retroceso...")
        self.publish_velocity(self.BACKUP_SPEED)
        time.sleep(0.3)

        self.get_logger().info("   ⏹️ Stop...")
        self.publish_velocity(0.0)
        time.sleep(0.2)

        self.get_logger().info(f"   🔪 Servo → {self.SERVO_CUT_ANGLE}°")
        self.publish_servo(self.SERVO_CUT_ANGLE)

        cut_msg = Bool()
        cut_msg.data = True
        self.cut_pub.publish(cut_msg)

        time.sleep(self.CUT_DURATION)

        self.get_logger().info(f"   ↪️ Servo → {self.SERVO_REST_ANGLE}°")
        self.publish_servo(self.SERVO_REST_ANGLE)
        time.sleep(0.3)

        self.get_logger().info("   ✅ Corte completado")
        self.is_cutting = False
        self.plants_cut += 1

    # ==================== IMAGE PROCESSING ====================

    def image_callback(self, ros_data):
        if not self.recorrido_activo or self.is_cutting:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
            self.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")

    def process_image(self, cv_image):
        self.frame_count += 1
        current_time = time.time()

        if current_time - self.last_detection_time < self.DETECTION_COOLDOWN:
            return

        (h, w) = cv_image.shape[:2]
        half_roi = self.ROI_SIZE // 2
        x1 = max(0, (w // 2) - half_roi)
        y1 = max(0, (h // 2) - half_roi)
        x2 = min(w, (w // 2) + half_roi)
        y2 = min(h, (h // 2) + half_roi)

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_bound = np.array([self.H_LOWER, self.S_LOWER, self.V_LOWER])
        upper_bound = np.array([self.H_UPPER, self.S_UPPER, self.V_UPPER])
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        fruit_detected = False
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > self.MIN_CONTOUR_AREA:
                fruit_detected = True

        if not fruit_detected:
            return

        # ===== FRUTA DETECTADA =====
        self.last_detection_time = current_time
        self.plants_checked += 1
        plant_number = self.plants_checked
        plant_name = f"{self.plant_name_base}_{plant_number}"

        roi_hsv = hsv_image[y1:y2, x1:x2]
        if roi_hsv.size == 0:
            return

        mean_hsv = cv2.mean(roi_hsv)
        h_val = int(mean_hsv[0])
        s_val = int(mean_hsv[1])
        v_val = int(mean_hsv[2])

        # Publicar nombre
        name_msg = String()
        name_msg.data = plant_name
        self.name_pub.publish(name_msg)

        # Publicar HSV
        self.publish_hsv(h_val, s_val, v_val)

        # Verificar madurez
        is_ripe = self.is_hsv_in_range(h_val, s_val, v_val)

        # Publicar chequeo (para sensor_logger)
        check_msg = Bool()
        check_msg.data = is_ripe
        self.check_pub.publish(check_msg)

        if is_ripe:
            self.get_logger().info(
                f"🍅 [{plant_number}/{self.TOTAL_PLANTS}] {plant_name} "
                f"HSV({h_val},{s_val},{v_val}) - MADURA → CORTANDO"
            )
            self.execute_cut_sequence()
        else:
            self.get_logger().info(
                f"🌿 [{plant_number}/{self.TOTAL_PLANTS}] {plant_name} "
                f"HSV({h_val},{s_val},{v_val}) - NO MADURA"
            )

        # Verificar si terminó
        if self.plants_checked >= self.TOTAL_PLANTS:
            self.finish_recorrido()

    def is_hsv_in_range(self, h, s, v):
        h_ok = self.H_LOWER <= h <= self.H_UPPER
        s_ok = self.S_LOWER <= s <= self.S_UPPER
        v_ok = self.V_LOWER <= v <= self.V_UPPER
        return h_ok and s_ok and v_ok

    def publish_hsv(self, h, s, v):
        h_msg = Int32()
        h_msg.data = h
        self.h_pub.publish(h_msg)

        s_msg = Int32()
        s_msg.data = s
        self.s_pub.publish(s_msg)

        v_msg = Int32()
        v_msg.data = v
        self.v_pub.publish(v_msg)

    # ==================== FINISH ====================

    def finish_recorrido(self):
        self.recorrido_activo = False
        self.publish_velocity(0.0)

        self.get_logger().info("")
        self.get_logger().info("=" * 55)
        self.get_logger().info("   🏁 RECORRIDO TERMINADO")
        self.get_logger().info("=" * 55)
        self.get_logger().info(f"   📊 Revisadas: {self.plants_checked}")
        self.get_logger().info(f"   ✂️  Cortadas:  {self.plants_cut}")
        self.get_logger().info(f"   ⏭️  Ignoradas: {self.plants_checked - self.plants_cut}")
        self.get_logger().info("=" * 55)

        # Publicar TERMINADO para mission_controller
        status_msg = String()
        status_msg.data = "TERMINADO"
        self.status_pub.publish(status_msg)

    def publish_status(self):
        msg = String()
        if self.recorrido_activo:
            msg.data = f"[HARVEST] {self.plants_checked}/{self.TOTAL_PLANTS} - Cut: {self.plants_cut}"
        elif self.plants_checked > 0:
            msg.data = "TERMINADO"
        else:
            msg.data = "[HARVEST] IDLE - Waiting for start signal"
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorROS2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_velocity(0.0)
        node.publish_servo(0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()