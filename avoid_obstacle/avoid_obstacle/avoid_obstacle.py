#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import signal
import sys
import time

class FollowWall(Node):
    def __init__(self):
        super().__init__('follow_wall')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub_laser = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile)

        self.timer = self.create_timer(0.1, self.control_loop)  # ✅ Timer added to call control_loop()

        self.laser_data = None
        self.state = 'go_forward_to_wall'
        self.turn_start_time = None

        # Parameters
        self.front_wall_threshold = 0.19       # Distance to detect wall in front
        self.wall_alignment_distance = 0.2    # Desired distance from the right wall
        self.too_close_distance = 0.15         # Too close to right wall
        self.wall_lost_threshold = 0.3         # No wall if right side is greater
        self.base_speed = 0.1                  # Forward speed
        self.turn_speed = 0.8                  # Turning speed

        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        self.get_logger().info('Shutting down...')
        rclpy.shutdown()
        sys.exit(0)

    def scan_cb(self, msg):
        self.laser_data = msg.ranges

    def get_valid_range(self, index):
        if 0 <= index < len(self.laser_data):
            val = self.laser_data[index]
            return val if not math.isinf(val) and not math.isnan(val) else float('inf')
        return float('inf')

    def control_loop(self):
        if not self.laser_data:
            return

        # front_dist = min(
        #     [self.get_valid_range(i) for i in range(330, 360)] +
        #     [self.get_valid_range(i) for i in range(0, 30)]
        # )
        front_ranges = self.laser_data[-1:] + self.laser_data[:1]
        self.minDistance = min(front_ranges)
        self.minDistance = max(self.minDistance, 0.01)
        front_dist = self.minDistance



        right_ranges = self.laser_data[250:260]
    
        right_ranges = max(right_ranges)
        right_ranges = min(right_ranges, 3.0)
        if right_ranges == 0.0:
            right_ranges = 3.0


        rightDistance = right_ranges

        right_ranges1 = self.laser_data[270:275]

        right_ranges1 = max(right_ranges1)
        right_ranges1 = min(right_ranges1, 3.0)
        if right_ranges1 == 0.0:
            right_ranges1 = 3.0


        rightDistance1 = right_ranges1

        right_ranges2 = self.laser_data[245:250]

        right_ranges2 = max(right_ranges2)
        right_ranges2 = min(right_ranges2, 3.0)
        if right_ranges2 == 0.0:
            right_ranges2 = 3.0


        rightDistanceToEvadeObstacle = right_ranges2

        self.get_logger().info(f'Front distance: {front_dist}, Right distance: {rightDistance}, Right distance 1: {rightDistance1}, Right distance 2: {rightDistanceToEvadeObstacle}')


        twist = Twist()

        if self.state == 'go_forward_to_wall':
            if front_dist < self.front_wall_threshold:
                twist.linear.x = 0.0
                self.get_logger().info('[STATE] Wall ahead detected — switching to turn left, and brakes applied')
                self.state = 'turn_left'
            else:
                self.get_logger().info('[STATE] Moving forward')
                twist.linear.x = self.base_speed

        elif self.state == 'turn_left':
            if rightDistance > self.wall_alignment_distance:
                #self.get_logger().info('[STATE] Too far from wall — turning left')
                twist.angular.z = self.turn_speed
            else:
                #self.get_logger().info('[STATE] Aligned with wall — moving forward')
                self.state = 'go_forward'

                twist.linear.x = self.base_speed

        elif self.state == 'align_with_wall':
            if rightDistance > self.wall_alignment_distance:
                #self.get_logger().info('[STATE] Too far from wall — turning left')
                twist.angular.z = self.turn_speed
            else:
                #self.get_logger().info('[STATE] Aligned with wall — moving forward')
                self.state = 'go_forward'

                # twist.linear.x = self.base_speed

        elif self.state == 'go_forward':
            if rightDistanceToEvadeObstacle < self.wall_alignment_distance:
                twist.linear.x = self.base_speed
            else:
                self.state = 'turn_right1'


        elif self.state == 'turn_right1':
            if rightDistance1 > self.wall_alignment_distance:
                twist.angular.z = -self.turn_speed
            else:
                self.state = 'turn_right2'
                

        elif self.state == 'turn_right2':
            if rightDistance1 < 0.5:
                twist.angular.z = -self.turn_speed
                

               
        self.pub.publish(twist)

    def move_forward(self):
        twist = Twist()
        twist.linear.x = self.base_speed
        twist.angular.z = 0.0
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FollowWall()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
