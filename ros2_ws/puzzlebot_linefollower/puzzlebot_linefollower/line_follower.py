import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        self.bridge = CvBridge()

        # Target (unsmoothed) and current (smooth) twists
        self.target_twist = Twist()
        self.current_twist = Twist()

        # Parameters you can tune
        self.publish_rate = 10        # Hz
        self.max_speed = 0.04         # m/s (safe speed)
        self.max_turn = 0.3           # rad/s
        self.accel_limit = 0.01       # maximum change per step

        # Subscribe to camera
        self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # Publish /cmd_vel at fixed rate
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(1.0 / self.publish_rate, self.publish_cmd)

        self.get_logger().info("Line follower with smoothing started.")

    def image_callback(self, msg):
        # Convert ROS Image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Blue mask
        lower_blue = np.array([90, 100, 50])
        upper_blue = np.array([120, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Centroid of the mask
        M = cv2.moments(mask)

        # ALWAYS define safe defaults
        vz = 0.0
        wz = 0.0

        # If enough blue is detected
        if M["m00"] > 20000:
            cx = int(M["m10"] / M["m00"])
            h, w = frame.shape[:2]

            error = cx - w // 2
            Kp = 0.0003

            wz = -Kp * error          # turn to center line
            vz = self.max_speed       # safe forward speed

            # Clamp turn speed
            wz = max(min(wz, self.max_turn), -self.max_turn)
        else:
            self.get_logger().warn("Line lost!")

        # Update target twist
        self.target_twist.linear.x = vz
        self.target_twist.angular.z = wz

    def smooth(self, current, target):
        """
        Limit acceleration: keep the change per cycle <= accel_limit.
        """
        if abs(target - current) > self.accel_limit:
            return current + self.accel_limit * np.sign(target - current)
        return target

    def publish_cmd(self):
        """
        Smooth current_twist toward target_twist and publish.
        """
        self.current_twist.linear.x = self.smooth(
            self.current_twist.linear.x,
            self.target_twist.linear.x
        )
        self.current_twist.angular.z = self.smooth(
            self.current_twist.angular.z,
            self.target_twist.angular.z
        )

        # Publish final command
        self.cmd_pub.publish(self.current_twist)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

