import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import cv2
from cv_bridge import CvBridge
import numpy as np

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from example_interfaces.srv import Trigger


class SimpleTunnelNavigator(Node):
    def __init__(self):
        super().__init__('simple_tunnel-navigator')

        main_timer_cb_group = MutuallyExclusiveCallbackGroup()
        laser_handler_cb_group = MutuallyExclusiveCallbackGroup()
        image_cb_group = MutuallyExclusiveCallbackGroup()
        service_cb_group = MutuallyExclusiveCallbackGroup()

        self.signDetected = False
        self.laser_data = None
        self.front_dist = None
        self.obstacle_detected = False
        self.canMove = False

        self.srv = self.create_service(
            Trigger,
            'detect_tunnel_sign',
            self.handle_detect_tunnel_sign,
            callback_group=service_cb_group
        )


        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub_laser = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile)

        image_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        default_qos = QoSProfile(depth=10)

        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, image_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', default_qos)
        self.timer = self.create_timer(0.1, self.main_timer_callback, callback_group=main_timer_cb_group)
        self.laser_handler_timer = self.create_timer(0.1, self.laser_handler, callback_group=laser_handler_cb_group)

        self.bridge = CvBridge()
        self.tunnel_detected = False

    def handle_detect_tunnel_sign(self, request, response):
        self.get_logger().info("Trigger service called to detect tunnel sign.")
        

        if self.signDetected :
            self.canMove = True
        response.success = self.signDetected
        response.message = "Sign detected." if self.signDetected else "No sign detected."
        
        return response


    def main_timer_callback(self):
        
        if self.canMove and not self.obstacle_detected:
            self.move_robot(0.1, 0.0)
        else:
            self.move_robot(0.0, 0.0)

    def scan_cb(self, msg):
        self.laser_data = msg.ranges

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        self.signDetected = self.detect_sign(cv_image)
        self.get_logger().info(f'Triangle sign detected: {self.signDetected}')

    def laser_handler(self):
        if not self.laser_data:
            return

        front_ranges = self.laser_data[-1:] + self.laser_data[:1]
        minDistance = min(front_ranges)
        minDistance = max(minDistance, 0.01)
        self.front_dist = minDistance

        if self.front_dist < 0.2:
            self.obstacle_detected = True

    def move_robot(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    def detect_sign(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])

        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower_red1, upper_red1),
            cv2.inRange(hsv, lower_red2, upper_red2)
        )
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        contours_red, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_yellow, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        debug_message = f"Red contours: {len(contours_red)}, Yellow contours: {len(contours_yellow)}."

        if not contours_red or not contours_yellow:
            return False

        for rc in contours_red:
            if len(cv2.approxPolyDP(rc, 0.04 * cv2.arcLength(rc, True), True)) != 3:
                continue

            for yc in contours_yellow:
                M = cv2.moments(yc)
                if M["m00"] == 0:
                    continue
                cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                if cv2.pointPolygonTest(rc, (cx, cy), False) >= 0:
                    return True

        return False


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = SimpleTunnelNavigator()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
