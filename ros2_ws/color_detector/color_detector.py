#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String

class ColorDetectorROS2(Node):
    def __init__(self):
        super().__init__("color_detector")

        # Parametros de configuracion para filtro de color y area
        self.declare_parameter('h_lower', 40)
        self.declare_parameter('s_lower', 100)
        self.declare_parameter('v_lower', 80)
        self.declare_parameter('h_upper', 80)
        self.declare_parameter('s_upper', 255)
        self.declare_parameter('v_upper', 255)
        self.declare_parameter('min_area', 500)
        self.declare_parameter('plant_name', 'Fondowny')

        self.H_LOWER = self.get_parameter('h_lower').value
        self.S_LOWER = self.get_parameter('s_lower').value
        self.V_LOWER = self.get_parameter('v_lower').value
        self.H_UPPER = self.get_parameter('h_upper').value
        self.S_UPPER = self.get_parameter('s_upper').value
        self.V_UPPER = self.get_parameter('v_upper').value
        self.MIN_CONTOUR_AREA = self.get_parameter('min_area').value
        self.plant_name = self.get_parameter('plant_name').value

        # Variables internas
        self.planta_ya_fue_cortada = False
        self.ROI_SIZE = 200
        self.frame_count = 0
        self.bridge = CvBridge()

        # Suscriptores y Publicadores
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

        # Timers de control
        self.create_timer(5.0, self.check_video_stream)
        self.create_timer(2.0, self.sync_plant_name)

        self.get_logger().info(f"Nodo iniciado. Planta objetivo: {self.plant_name}")

    def sync_plant_name(self):
        """Publica periodicamente el nombre de la planta para sincronizacion con ESP32."""
        msg = String()
        msg.data = self.plant_name
        self.name_pub.publish(msg)

    def check_video_stream(self):
        """Verifica la recepcion de flujo de video."""
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
        
        # Definicion de ROI central
        (h, w) = cv_image.shape[:2]
        half_roi = self.ROI_SIZE // 2
        x1 = max(0, (w // 2) - half_roi)
        y1 = max(0, (h // 2) - half_roi)
        x2 = min(w, (w // 2) + half_roi)
        y2 = min(h, (h // 2) + half_roi)

        # Procesamiento HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_bound = np.array([self.H_LOWER, self.S_LOWER, self.V_LOWER])
        upper_bound = np.array([self.H_UPPER, self.S_UPPER, self.V_UPPER])
        
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Calculo y publicacion de valores promedio en ROI
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

        # Deteccion de contornos para accion de corte
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours and not self.planta_ya_fue_cortada:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > self.MIN_CONTOUR_AREA:
                self.get_logger().info(f"Objetivo detectado (Area: {area:.0f}). Enviando senal de corte.")
                
                self.planta_ya_fue_cortada = True
                
                cut_msg = Bool()
                cut_msg.data = True
                self.cut_pub.publish(cut_msg)

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
