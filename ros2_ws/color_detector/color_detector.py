import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import Twist

class ColorDetectorROS2(Node):
    def __init__(self):
        super().__init__("color_detector")

        self.declare_parameter('h_lower', 0)
        self.declare_parameter('s_lower', 100)
        self.declare_parameter('v_lower', 100)
        self.declare_parameter('h_upper', 10)
        self.declare_parameter('s_upper', 255)
        self.declare_parameter('v_upper', 255)
        self.declare_parameter('min_area', 500)
        self.declare_parameter('plant_name', 'Planta_1')

        self.H_LOWER = self.get_parameter('h_lower').value
        self.S_LOWER = self.get_parameter('s_lower').value
        self.V_LOWER = self.get_parameter('v_lower').value
        self.H_UPPER = self.get_parameter('h_upper').value
        self.S_UPPER = self.get_parameter('s_upper').value
        self.V_UPPER = self.get_parameter('v_upper').value
        self.MIN_CONTOUR_AREA = self.get_parameter('min_area').value
        self.plant_name = self.get_parameter('plant_name').value

        self.planta_ya_fue_cortada = False
        self.ROI_SIZE = 200
        self.frame_count = 0
        self.bridge = CvBridge()
        self.last_hsv_publish_time = self.get_clock().now()

        self.image_sub = self.create_subscription(
            Image,
            "/video_source/raw",
            self.image_callback,
            10
        )

        self.h_pub = self.create_publisher(Int32, "hsv_h", 10)
        self.s_pub = self.create_publisher(Int32, "hsv_s", 10)
        self.v_pub = self.create_publisher(Int32, "hsv_v", 10)
        self.cut_pub = self.create_publisher(Bool, "cut_plant", 10)
        self.name_pub = self.create_publisher(String, "plant_name", 10)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.forward_twist = Twist()
        self.forward_twist.linear.x = 0.01

        self.movement_timer = self.create_timer(0.1, self.publish_forward_velocity)

        self.create_timer(5.0, self.check_video_stream)
        self.create_timer(2.0, self.sync_plant_name)

    def publish_forward_velocity(self):
        if not self.planta_ya_fue_cortada:
            self.cmd_pub.publish(self.forward_twist)

    def stop_robot(self):
        stop = Twist()
        self.cmd_pub.publish(stop)

    def sync_plant_name(self):
        msg = String()
        msg.data = self.plant_name
        self.name_pub.publish(msg)

    def check_video_stream(self):
        if self.frame_count == 0:
            self.get_logger().warn("No se detecta flujo de video en /video_source/raw")

    def image_callback(self, ros_data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
            self.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f"Fallo en conversion cv_bridge: {e}")

    def process_image(self, cv_image):
        self.frame_count += 1

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

        target_detected = False
        largest_contour = None

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > self.MIN_CONTOUR_AREA:
                target_detected = True

        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_hsv_publish_time).nanoseconds / 1e9

        if time_diff >= 15.0 or target_detected:
            roi_hsv = hsv_image[y1:y2, x1:x2]
            if roi_hsv.size != 0:
                mean = cv2.mean(roi_hsv)
                h_msg = Int32()
                h_msg.data = int(mean[0])
                self.h_pub.publish(h_msg)
                s_msg = Int32()
                s_msg.data = int(mean[1])
                self.s_pub.publish(s_msg)
                v_msg = Int32()
                v_msg.data = int(mean[2])
                self.v_pub.publish(v_msg)
                self.last_hsv_publish_time = current_time

        if target_detected and not self.planta_ya_fue_cortada:
            self.planta_ya_fue_cortada = True

            backward = Twist()
            backward.linear.x = -0.005
            self.cmd_pub.publish(backward)

            stop = Twist()
            self.cmd_pub.publish(stop)

            cut_msg = Bool()
            cut_msg.data = True
            self.cut_pub.publish(cut_msg)

            self.movement_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorROS2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

