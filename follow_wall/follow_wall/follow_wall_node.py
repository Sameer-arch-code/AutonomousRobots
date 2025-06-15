#!/usr/bin/env python3

import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class follow_wall_node(Node):
    def __init__(self):
        super().__init__('follow_wall_node')

        move_turtle_cb_group = MutuallyExclusiveCallbackGroup()
        stop_turtle_cb_group = MutuallyExclusiveCallbackGroup()
        laser_cb_group = MutuallyExclusiveCallbackGroup()
        odom_cb_group = MutuallyExclusiveCallbackGroup()
        wall_tracker_cb_group = MutuallyExclusiveCallbackGroup()
        
        # Create a publisher for the /turtle1/cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.scanData = None
        self.odomData = None
        self.minDistance = None
        self.obstacleDetected = False
        self.firstRightTurn = False
        self.reachedRightWall = True
        self.hasReachedFrontWall = False
        
        self.obstacleEvaded = True
        self.orientation = "Up"
        self.obstacleTurnFlag=False
        self.subscriber = self.create_subscription(LaserScan, '/scan',self.scanCallBack,1, callback_group = laser_cb_group)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom',self.odom_callback,1, callback_group = odom_cb_group)


        self.timer = self.create_timer(0.5, self.mainTimer, callback_group = move_turtle_cb_group)
        self.timer1 = self.create_timer(0.01, self.obstacleDetector, callback_group = stop_turtle_cb_group)
        
        #self.timer1 = self.create_timer(timer_period, self.countTurn,callback_group=turn_cb_group)
        self.get_logger().info("Turtle mover node has been started")

    def mainTimer(self):
        if self.scanData is None or not self.scanData.ranges:
            self.get_logger().warn("No valid LaserScan data received yet.")
            return
        self.get_logger().info(f"orientation:{self.orientation}")
        if not self.hasReachedFrontWall :
            self.attachToFrontWall()
        if not self.reachedRightWall:
            self.attachToRightWall()
        if self.reachedRightWall and self.hasReachedFrontWall:
            if not self.obstacleDetected:
                if self.rightDistance <0.5 :
                    self.driveForward()
                    self.obstacleTurnFlag =False

                elif self.obstacleTurnFlag ==True:
                    self.get_logger().warn("obstacle turn flag is True")
                    self.driveForward()

                
                elif self.obstacleEvaded:
                    self.obstacleEvaded = False
                    self.evadeObstacle()
                    self.obstacleTurnFlag = True
            if self.obstacleDetected:
                match self.orientation:
                    case "Up":
                        self.turnLeft()
                    case "Right":
                        self.turnUpFromRight()
                    case "Left":
                        self.turnDownFromLeft()
                    case "Down":
                        self.turnRightFromDown()
            


        # if not self.obstacleDetected and self.reachedRightWall:
        #     self.driveForward()
        # else:
        #     self.turnLeft()

    


    def evadeObstacle(self):
        if self.obstacleEvaded == False:
            match self.orientation:
                case "Up":
                    self.turnRight()
                case "Right":
                    self.turnDownFromRight()
                case "Left":
                    self.turnUpFromLeft()
                case "Down":
                    self.turnLeftFromDown()
            self.obstacleEvaded = True
        
             

    def attachToRightWall(self):
        self.get_logger().warn("attaching to right wall")
        if not self.firstRightTurn:
            self.turnRight()
            self.firstRightTurn = True

        if not self.obstacleDetected:
            self.get_logger().warn("drive forward must run")
            self.driveForward()
        else:
            self.turnUpFromRight()
            self.reachedRightWall = True

    def attachToFrontWall(self):
        self.get_logger().warn("attaching to front wall")
        if not self.obstacleDetected:
            self.get_logger().warn("drive forward must run")
            self.driveForward()
        else:
            self.hasReachedFrontWall = True

    def obstacleDetector(self):

        if self.scanData is None or not self.scanData.ranges:
            self.get_logger().warn("No valid LaserScan data received yet.")
            return

        # self.frontDistance = self.scanData.ranges[0]
        
        right_ranges = self.scanData.ranges[255:260]
        right_ranges = [3.5 if math.isinf(r) else r for r in right_ranges]

        self.rightDistance = max(right_ranges)

        #self.get_logger().info(f"rightDistance:{self.rightDistance}")

        # for i, distance in enumerate(self.scanData.ranges):
        #     self.get_logger().info(f"Range[{i}]: {distance}")


        front_ranges = self.scanData.ranges[-8:] + self.scanData.ranges[:8]
        self.minDistance = min(front_ranges)

        if self.minDistance <= 0.5:
            if self.obstacleDetected == False:
                self.obstacleDetected = True
                self.handBreak()
        else:
            self.obstacleDetected = False

        # if self.rightDistance <= 0.55:
        #     if self.obstacleDetected == False:
        #         self.obstacleDetected = True
        #         msg = Twist()
        #         msg.linear.x =0.0
        #         self.cmd_vel_pub.publish(msg)
    def handBreak(self):
        msg = Twist()
        msg.linear.x =0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def driveForward(self):
        msg = Twist()
        if self.minDistance>1.5:
            msg.linear.x = 0.35
        else:
            msg.linear.x = 0.3 
        self.cmd_vel_pub.publish(msg)

    def turnLeft(self):

        quat = self.odomData.pose.pose.orientation
        
        euler = self.euler_from_quaternion(quat)
        yaw = euler[2]
        self.get_logger().info(f"orientationYaw:{yaw}")

        while yaw <= 70.0:
            # self.get_logger().info(f"orientationYaw:{yaw}")
            # self.get_logger().info("turn left is active")
            msg = Twist()
            msg.angular.z =1.5
            self.cmd_vel_pub.publish(msg)
            
            quat = self.odomData.pose.pose.orientation
            euler = self.euler_from_quaternion(quat)
            yaw = euler[2]

        while yaw <= 90.0:
            # self.get_logger().info(f"orientationYaw:{yaw}")
            # self.get_logger().info("turn left is active")
            msg = Twist()
            msg.angular.z =0.08
            self.cmd_vel_pub.publish(msg)
            
            quat = self.odomData.pose.pose.orientation
            euler = self.euler_from_quaternion(quat)
            yaw = euler[2]
        self.get_logger().info("end of while loop")
        self.get_logger().info(f"orientationYaw:{yaw}")
        self.handBreak()
        self.orientation = "Left"

    def turnLeftFromDown(self):

        quat = self.odomData.pose.pose.orientation
        
        euler = self.euler_from_quaternion(quat)
        yaw = euler[2]
        if yaw <0.0:
            yaw+=360
        self.get_logger().warn(f"orientationYaw:{yaw}")

        while yaw > 100.0:
            # self.get_logger().info(f"orientationYaw:{yaw}")
            # self.get_logger().info("turn left is active")
            msg = Twist()
            msg.angular.z = -1.5
            self.cmd_vel_pub.publish(msg)
            
            quat = self.odomData.pose.pose.orientation
            euler = self.euler_from_quaternion(quat)
            yaw = euler[2]
            if yaw <0.0:
                yaw+=360
            self.get_logger().warn(f"yaw while turnLeftFromDown Fast:{yaw}")

        while yaw > 90.0:
            # self.get_logger().info(f"orientationYaw:{yaw}")
            # self.get_logger().info("turn left is active")
            msg = Twist()
            msg.angular.z = -0.08
            self.cmd_vel_pub.publish(msg)
            
            quat = self.odomData.pose.pose.orientation
            euler = self.euler_from_quaternion(quat)
            yaw = euler[2]
            if yaw <0.0:
                yaw+=360
            self.get_logger().warn(f"yaw while turnLeftFromDown:{yaw}")
        self.get_logger().warn("end of while loop")
        self.get_logger().warn(f"yaw after turnLeftFromDown:{yaw}")
        self.handBreak()
        self.orientation = "Left"
    
    def turnRightFromDown(self):

        quat = self.odomData.pose.pose.orientation
        
        euler = self.euler_from_quaternion(quat)
        yaw = euler[2]
        if yaw <0.0:
            yaw+=360.0
        self.get_logger().info(f"orientationYaw:{yaw}")

        while yaw <= 250.0:
            # self.get_logger().info(f"orientationYaw:{yaw}")
            # self.get_logger().info("turn left is active")
            msg = Twist()
            msg.angular.z = 1.5
            self.cmd_vel_pub.publish(msg)
            
            quat = self.odomData.pose.pose.orientation
            euler = self.euler_from_quaternion(quat)
            yaw = euler[2]
            if yaw <0.0:
                yaw+=360.0

        while yaw <= 270.0:
            # self.get_logger().info(f"orientationYaw:{yaw}")
            # self.get_logger().info("turn left is active")
            msg = Twist()
            msg.angular.z = 0.08
            self.cmd_vel_pub.publish(msg)
            
            quat = self.odomData.pose.pose.orientation
            euler = self.euler_from_quaternion(quat)
            yaw = euler[2]
            if yaw <0.0:
                yaw+=360.0
        self.get_logger().info("end of while loop")
        self.get_logger().info(f"orientationYaw:{yaw}")
        self.handBreak()
        self.orientation = "Right"

    def turnRight(self): 
        quat = self.odomData.pose.pose.orientation
        
        euler = self.euler_from_quaternion(quat)
        yaw = euler[2]
        self.get_logger().info(f"orientationYaw:{yaw}")

        while yaw >= -70.0:
            # self.get_logger().info(f"orientationYaw:{yaw}")
            # self.get_logger().info("turn right is active")
            msg = Twist()
            msg.angular.z =-1.5
            self.cmd_vel_pub.publish(msg)
            
            quat = self.odomData.pose.pose.orientation
            euler = self.euler_from_quaternion(quat)
            yaw = euler[2]

        while yaw >= -90.0:
            # self.get_logger().info(f"orientationYaw:{yaw}")
            # self.get_logger().info("turn right is active")
            msg = Twist()
            msg.angular.z =-0.08
            self.cmd_vel_pub.publish(msg)
            
            quat = self.odomData.pose.pose.orientation
            euler = self.euler_from_quaternion(quat)
            yaw = euler[2]
        self.get_logger().info("end of while loop")
        self.get_logger().info(f"orientationYaw:{yaw}")
        self.handBreak()
        self.orientation = "Right"

    def turnUpFromRight(self):
        quat = self.odomData.pose.pose.orientation
        
        euler = self.euler_from_quaternion(quat)
        yaw = euler[2]
        self.get_logger().info(f"orientationYaw:{yaw}")

        while yaw <= -20:
            # self.get_logger().info(f"orientationYaw:{yaw}")
            # self.get_logger().info("turnFrontFromRight is active")
            msg = Twist()
            msg.angular.z =1.5
            self.cmd_vel_pub.publish(msg)
            
            quat = self.odomData.pose.pose.orientation
            euler = self.euler_from_quaternion(quat)
            yaw = euler[2]

        while yaw <= 0:
            # self.get_logger().info(f"orientationYaw:{yaw}")
            # self.get_logger().info("turnFrontFromRight is active")
            msg = Twist()
            msg.angular.z =0.08
            self.cmd_vel_pub.publish(msg)
            
            quat = self.odomData.pose.pose.orientation
            euler = self.euler_from_quaternion(quat)
            yaw = euler[2]
        self.get_logger().info("end of while loop")
        self.get_logger().info(f"orientationYaw:{yaw}")
        self.handBreak()
        self.orientation = "Up"

    def turnUpFromLeft(self):

        quat = self.odomData.pose.pose.orientation
        
        euler = self.euler_from_quaternion(quat)
        yaw = euler[2]
        self.get_logger().info(f"orientationYaw:{yaw}")

        while yaw >= 20:
            # self.get_logger().info(f"orientationYaw:{yaw}")
            # self.get_logger().info("turnFrontFromLeft is active")
            msg = Twist()
            msg.angular.z = -1.5
            self.cmd_vel_pub.publish(msg)
            
            quat = self.odomData.pose.pose.orientation
            euler = self.euler_from_quaternion(quat)
            yaw = euler[2]

        while yaw >= 0:
            # self.get_logger().info(f"orientationYaw:{yaw}")
            # self.get_logger().info("turnFrontFromLeft is active")
            msg = Twist()
            msg.angular.z = -0.08
            self.cmd_vel_pub.publish(msg)
            
            quat = self.odomData.pose.pose.orientation
            euler = self.euler_from_quaternion(quat)
            yaw = euler[2]
        self.get_logger().info("end of while loop")
        self.get_logger().info(f"orientationYaw:{yaw}")
        self.handBreak()
        self.orientation = "Up"

    def turnDownFromLeft(self):

        quat = self.odomData.pose.pose.orientation
        
        euler = self.euler_from_quaternion(quat)
        yaw = euler[2]
        if yaw<0.0:
            yaw += 360.0
        self.get_logger().info(f"orientationYaw before turnDownFromLeft:{yaw}")

        while yaw < 160.0:
            # self.get_logger().info(f"orientationYaw:{yaw}")
            # self.get_logger().info("turnDownkFromLeft is active")
            msg = Twist()
            msg.angular.z = 1.5
            self.cmd_vel_pub.publish(msg)
            
            quat = self.odomData.pose.pose.orientation
            euler = self.euler_from_quaternion(quat)
            yaw = euler[2]
        while yaw < 180.0:
            # self.get_logger().info(f"orientationYaw:{yaw}")
            # self.get_logger().info("turnDownkFromLeft is active")
            msg = Twist()
            msg.angular.z = 0.08
            self.cmd_vel_pub.publish(msg)
            
            quat = self.odomData.pose.pose.orientation
            euler = self.euler_from_quaternion(quat)
            yaw = euler[2]
            if yaw<0.0:
                yaw += 360.0
        self.get_logger().info("end of while loop")
        self.get_logger().info(f"orientationYaw after turnDownFromLeft:{yaw}")
        self.handBreak()
        self.orientation = "Down"

    def turnDownFromRight(self):

        quat = self.odomData.pose.pose.orientation
        
        euler = self.euler_from_quaternion(quat)
        yaw = euler[2]
        if yaw <0.0:
            yaw+=360

        self.get_logger().info(f"orientationYaw:{yaw}")

        while yaw > 200.0:
            # self.get_logger().info(f"orientationYaw:{yaw}")
            # self.get_logger().info("turnFrontFromLeft is active")
            msg = Twist()
            msg.angular.z = -1.5
            self.cmd_vel_pub.publish(msg)
            
            quat = self.odomData.pose.pose.orientation
            euler = self.euler_from_quaternion(quat)
            yaw = euler[2]
            if yaw <0.0:
                yaw+=360

        while yaw > 180.0:
            # self.get_logger().info(f"orientationYaw:{yaw}")
            # self.get_logger().info("turnFrontFromLeft is active")
            msg = Twist()
            msg.angular.z = -0.08
            self.cmd_vel_pub.publish(msg)
            
            quat = self.odomData.pose.pose.orientation
            euler = self.euler_from_quaternion(quat)
            yaw = euler[2]
            if yaw <0.0:
                yaw+=360
        self.get_logger().info("end of while loop")
        self.get_logger().info(f"orientationYaw:{yaw}")
        self.handBreak()
        self.orientation = "Down"




    def scanCallBack(self,msg):
        self.scanData = msg
    def odom_callback(self,msg):
        self.odomData = msg

    def euler_from_quaternion(self,quaternion):

        #source: https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        

        # Convert results to degrees
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)

        

        return roll_deg, pitch_deg, yaw_deg

    def quaternion_from_euler(self,euler_angles):
        #source: https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
        """
        Converts Euler angles (roll, pitch, yaw) in degrees to a quaternion (x, y, z, w).
    
        Args:
            roll: Rotation around the X-axis in degrees.
            pitch: Rotation around the Y-axis in degrees.
            yaw: Rotation around the Z-axis in degrees.
    
        Returns:
            A list [x, y, z, w] representing the quaternion.
        """

        # Extract roll, pitch, and yaw from the input list
        roll, pitch, yaw = euler_angles
        # Convert degrees to radians
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)

        # Perform the quaternion calculations
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        # Ensure the quaternion is in [x, y, z, w] order
        q = [0] * 4
        q[0] = cr * cp * cy + sr * sp * sy  # w
        q[1] = sr * cp * cy - cr * sp * sy  # x
        q[2] = cr * sp * cy + sr * cp * sy  # y
        q[3] = cr * cp * sy - sr * sp * cy  # z

        #    Swap to [x, y, z, w]
        return [q[1], q[2], q[3], q[0]]
        


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = follow_wall_node()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

