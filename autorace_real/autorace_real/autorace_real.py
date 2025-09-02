import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from geometry_msgs.msg import Twist

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from turtlesim.msg import Pose

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time


linearSpeed = 0.15
# turnLinearSpeed = 0.021
#turnLinearSpeed = 0.03
turnLinearSpeed = 0.06
turnLinearSpeedSlow = 0.035

#blindTurnLinearSpeed =0.02

blindTurnLinearSpeed =0.07
blindTurnAngularSpeed = 0.9
# blindTurnLinearSpeed =0.02

blindMoveForwardSpeed = 0.2
KP = 1/1000




class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')

        main_timer_cb_group = MutuallyExclusiveCallbackGroup()
        cv_cb_group = MutuallyExclusiveCallbackGroup()
        laser_cb_group = MutuallyExclusiveCallbackGroup()
        odom_cb_group = MutuallyExclusiveCallbackGroup()
        stop_turtle_cb_group = MutuallyExclusiveCallbackGroup()

        self.current_frame = None
        self.error = None
        self.error2 = None
        self.errorExit = None

        self.turnDetected = False

        self.scanData = None
        self.laser_data = None
        self.odomData = None
        self.minDistance = None
        self.obstacleDetected = True
        self.oldAngularZ = None
        self.canBlindlyMoveForward = True
        self.jerkCount = 0
        self.move_forward_start_time = None

        self.flip = True
        self.isErrorNegative = True
        self.lineFound = False
        self.yellow_mask_to_evade_obstacle = None
        self.rightTurnCount =0
        self.imageWidth = None
        self.imageHeight = None
        self.pringleDetected = False
        self.searchRightWallFlag = False
        self.rightObstacleDetected = False

        # PID constants (tune these!)
        self.Kp = 0.0009
        # self.Kp_high = 0.002
        self.Kp_high = 0.003
        self.Ki = 0.0000
        self.Kd = 0.0005


        # PID state
        self.prev_error = 0.0
        self.integral = 0.0

        
        self.prev_time = self.get_clock().now()
        self.turn_speed = 0.5
        self.alignment_threshold = 0.19  # wall/box detection distance at 60 degree
        self.wall_alignment_threshold = 0.27 # 
        #self.wall_allignment_buffer = 0.06
        self.wall_allignment_buffer = 0.08
        self.base_speed = 0.08



        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscriber = self.create_subscription(LaserScan, '/scan',self.scanCallBack, qos_profile)


        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.br = CvBridge()

        self.timer = self.create_timer(0.1, self.mainTimer, callback_group = main_timer_cb_group)
        
        self.cv_timerYellow = self.create_timer(0.1, self.lineDetectionYellow, callback_group = cv_cb_group)
        

        self.timer1 = self.create_timer(0.01, self.obstacleDetector, callback_group = stop_turtle_cb_group)

        self.state = 'waitForStartSignal'
        #self.state = 'follow_right_wall'


        self.signDetected = False
        image_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, image_qos)

        

    def mainTimer(self):
        twist = Twist()   # for avoid obstacle

        if self.state == 'followLine':
            errorUsed = self.error
            if errorUsed is not None:
                if self.obstacleDetected and abs(errorUsed) <=40: 
                        self.state = 'turnRight'
                        self.turn_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                        self.handBreak()
                        return

            errorUsedForSpeed = self.error       
            if not self.obstacleDetected:
                msg = Twist()
                if errorUsed is not None:
                    
                    if errorUsed<0:
                        self.isErrorNegative = True
                    else:
                        self.isErrorNegative = False

                    self.jerkCount = 0
                    
                    self.get_logger().info(f"rotation error:{errorUsed}")
                    
                
                    if -140 < errorUsed < -110 and self.canBlindlyMoveForward:
                        msg.linear.x = blindMoveForwardSpeed 
                        self.get_logger().error("blind forward active")
                    else:
                        self.oldAngularZ = -self.rotationController(errorUsed)
                        msg.angular.z = -self.rotationController(errorUsed)

                    if -30 < errorUsed < 30 :
                        self.canBlindlyMoveForward = True

                if errorUsed is None:
                    self.canBlindlyMoveForward = False

                    if self.signDetected:
                        self.state = 'enter_tunnel'
                        self.handBreak()
                        self.turn_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                        return
            

                    if self.flip and self.jerkCount <= 8:
                        self.get_logger().info("blind jerk active")
                        msg.linear.x = 0.1
                        self.flip = False
                        self.jerkCount +=1
                    else: 
                        if self.isErrorNegative:
                            self.get_logger().error("blind left turn active")
                            msg.linear.x = blindTurnLinearSpeed 
                            msg.angular.z = blindTurnAngularSpeed
                        else:
                            self.get_logger().error("blind right turn active")
                            msg.linear.x = -blindTurnLinearSpeed 
                            msg.angular.z = -blindTurnAngularSpeed

                        self.flip = True
                        
                if errorUsedForSpeed is not None:   
                    scale = 1
    
                    if self.turnDetected:
                        if self.signDetected:
                            msg.linear.x = scale * turnLinearSpeedSlow
                        else:
                            msg.linear.x = scale * turnLinearSpeed
                    else:
                        if self.signDetected:
                            msg.linear.x = scale * turnLinearSpeedSlow
                        else:
                            msg.linear.x = scale * linearSpeed

                

                self.cmd_vel_pub.publish(msg)

        if self.state == 'followLineExit':
            errorUsed = self.errorExit
            if errorUsed is None:
                self.get_logger().info('error exit is none . Switching state to follow right wall')
                self.state = 'follow_right_wall'
                return
            if errorUsed is not None:
                if self.obstacleDetected and abs(errorUsed) <=40: 
                        self.state = 'turnRight'
                        self.turn_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                        self.handBreak()
                        return

            errorUsedForSpeed = self.errorExit       
            if not self.obstacleDetected:
                msg = Twist()
                if errorUsed is not None:
                    
                    if errorUsed<0:
                        self.isErrorNegative = True
                    else:
                        self.isErrorNegative = False

                    self.jerkCount = 0
                    
                    self.get_logger().info(f"rotation error:{errorUsed}")
                    
                
                    if -140 < errorUsed < -110 and self.canBlindlyMoveForward:
                        msg.linear.x = blindMoveForwardSpeed 
                        self.get_logger().error("blind forward active")
                    else:
                        self.oldAngularZ = -self.rotationController(errorUsed)
                        msg.angular.z = -self.rotationController(errorUsed)

                    if -30 < errorUsed < 30 :
                        self.canBlindlyMoveForward = True

                if errorUsed is None:
                    self.canBlindlyMoveForward = False

                    if self.signDetected:
                        self.state = 'enter_tunnel'
                        self.handBreak()
                        self.turn_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                        return
            

                    if self.flip and self.jerkCount <= 8:
                        self.get_logger().info("blind jerk active")
                        msg.linear.x = 0.1
                        self.flip = False
                        self.jerkCount +=1
                    else: 
                        if self.isErrorNegative:
                            self.get_logger().error("blind left turn active")
                            msg.linear.x = blindTurnLinearSpeed 
                            msg.angular.z = blindTurnAngularSpeed
                        else:
                            self.get_logger().error("blind right turn active")
                            msg.linear.x = -blindTurnLinearSpeed 
                            msg.angular.z = -blindTurnAngularSpeed

                        self.flip = True
                        
                if errorUsedForSpeed is not None:   
                    scale = 1
    
                    if self.turnDetected:
                        if self.signDetected:
                            msg.linear.x = scale * turnLinearSpeedSlow
                        else:
                            msg.linear.x = scale * turnLinearSpeed
                    else:
                        if self.signDetected:
                            msg.linear.x = scale * turnLinearSpeedSlow
                        else:
                            msg.linear.x = scale * linearSpeed

                

                self.cmd_vel_pub.publish(msg)
        if self.state == 'waitForStartSignal':   
            if self.obstacleDetected == True:
                self.get_logger().info("obstacle detected!! waiting for it to be removed")
            else:
                self.get_logger().warn("obstacle removed. changing state to 'followLine'")
                self.state = 'followLine'
        if self.state == 'turnRight':
            self.get_logger().info("turn right started and running")
            if not self.laser_data:
                return
            front_range = min(self.get_valid_range(i) for i in range(350, 360))  # Front
            range_60 = self.get_valid_range(55)  # Left-rear side (60° left)

            
            self.get_logger().info(f'[SENSOR] Front = {front_range:.2f} m | 60° = {range_60:.2f} m | State = {self.state}')

            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if current_time - self.turn_start_time < 3.25:  
                twist.angular.z = -self.turn_speed
                self.get_logger().info('[STATE] Turning right to face along wall')
            else:
                self.get_logger().info('[STATE] Done turning -- switching state')
                self.rightTurnCount +=1
                if self.rightTurnCount % 2 == 0:
                    self.state = 'followLine'
                elif self.searchRightWallFlag:
                    self.state ='search_right_wall'
                else:
                    self.state = 'search_wall'

            self.cmd_vel_pub.publish(twist)

        if self.state == 'search_wall':
            if not self.laser_data:
                return

            if (self.searchLine()):
                self.turn_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                self.state = 'turnRight'
                return

            front_range = min(self.get_valid_range(i) for i in range(350, 360))  # Front
            range_60 = self.get_valid_range(50)  # Left side (60° left)
            
            if range_60 == 0.0:
                range_60 = 10.0
            if range_60 == float('inf') or range_60 > self.alignment_threshold:
                twist.angular.z = self.turn_speed
                self.get_logger().info('[STATE] No wall/box at 60° — turning left')
            else:
                self.get_logger().warn(f'range_60 :{range_60} under threashold — switching to follow_wall')
                self.state = 'follow_wall'
            self.cmd_vel_pub.publish(twist)


        if self.state == 'follow_wall':
            if not self.laser_data:
                return

            if (self.searchLine()):
                self.turn_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                self.state = 'turnRight'
                return

            front_range = min(self.get_valid_range(i) for i in range(350, 360))  # Front
            range_60 = self.get_valid_range(50)  # Left side (60° left)

            if range_60 == 0.0:
                range_60 = 10.0

            if range_60 != float('inf') and range_60 <= self.alignment_threshold:
                twist.linear.x = self.base_speed
                self.get_logger().info(f'range_60:{range_60} is under threashold — moving forward')

            else:
                self.get_logger().warn(f'[STATE] Lost wall/box with a big range_60 : {range_60} — switching to search')
                self.state = 'search_wall'
            self.cmd_vel_pub.publish(twist)

        if self.state == 'turn_left':
        
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if current_time - self.turn_start_time < 3.5:  
                twist.angular.z = self.turn_speed
                
            else:
                self.get_logger().error('turn left is over, switching to move forward a little')
                self.state = 'move_forward_a_little'
                self.handBreak()
                return
            self.cmd_vel_pub.publish(twist)
                
        if self.state == 'move_forward_a_little':
            if self.rightObstacleDetected:
                self.get_logger().error('right obstacle found, switching state to follow right wall')
                #self.get_logger().error(f"right distance:{self.rightDistance} is less than 0.38")

                self.state = 'follow_right_wall'
            else:
                self.get_logger().info('right obstacle not found, switching state to evade pringle')
                self.state = 'evade_pringle'
        if self.state == 'evade_pringle':
            if self.pringleDetected:
                self.get_logger().info('pringle/obstacle detected again, switching state to turn_left')
                self.turn_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                self.state = 'turn_left'
                return
            else:
                if self.errorExit is not None:
                    self.get_logger().error('error exit found, changing state to followLineExit ')
                    self.state = 'followLineExit'
                    return

                self.get_logger().error('evading pringle ')
                twist.angular.z = -self.turn_speed+0.2
                twist.linear.x = self.base_speed-0.01 
            self.cmd_vel_pub.publish(twist)

            



        if self.state == 'follow_right_wall':
            if not self.laser_data:
                return
            if self.errorExit is not None:
                self.get_logger().error('error exit found, swithing state to followLineExit')
                self.state = 'followLineExit'
            if self.error is not None:
                self.state = 'followLine'
                return

            range_60 = self.get_valid_range(310)  # right side

            if range_60 == 0.0:
                range_60 = 10.0
            
            if self.pringleDetected:
                self.get_logger().info('obstacle detected. Switching state to turn left')
                self.turn_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                self.state = 'turn_left'
                return


            if range_60 != float('inf') and range_60 <= self.wall_alignment_threshold and range_60 != float('inf') and range_60 >= self.wall_alignment_threshold - self.wall_allignment_buffer:
                twist.linear.x = self.base_speed +0.1
                

            elif range_60 != float('inf') and range_60 >= self.wall_alignment_threshold:
                
                self.state = 'search_right_wall'
            elif range_60 != float('inf') and range_60 <= self.wall_alignment_threshold - self.wall_allignment_buffer:
                
                self.state = 'anti_search_right_wall'

            self.cmd_vel_pub.publish(twist)

        if self.state == 'search_right_wall':
            if not self.laser_data:
                return

            
            range_60 = self.get_valid_range(310)  # Left side (60° left)
            
            if range_60 == 0.0:
                range_60 = 10.0
            if range_60 == float('inf') or range_60 > self.wall_alignment_threshold:
                self.get_logger().info('turning right for wall')
                twist.angular.z = -self.turn_speed+0.3
                
            else:
                
                self.state = 'follow_right_wall'
            self.cmd_vel_pub.publish(twist)

        if self.state == 'anti_search_right_wall':
            if not self.laser_data:
                return

            
            range_60 = self.get_valid_range(310)  # Left side (60° left)
            
            if range_60 == 0.0:
                range_60 = 10.0
            if range_60 == float('inf') or range_60 < self.wall_alignment_threshold - self.wall_allignment_buffer:
                self.get_logger().info('turning left from wall')
                twist.angular.z = self.turn_speed-0.3
            
            else:
            
                self.state = 'follow_right_wall'
            self.cmd_vel_pub.publish(twist)

        if self.state == 'enter_tunnel':
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if current_time - self.turn_start_time < 5.0:  
                twist.linear.x = self.base_speed
                self.get_logger().error('entering tunnel')
            else:
                self.state = 'follow_right_wall'

            self.cmd_vel_pub.publish(twist)


    def searchLine(self):
        # Pick scanline to evade obstacle
        bottom_percentage_evade_obstacle_scan = 5
        right_percent = 35
        
        self.yellow_mask_to_evade_obstacle[:, int(self.imageWidth * (1 - right_percent / 100)):] = 0

        left_percent = 50

        self.yellow_mask_to_evade_obstacle[:, :int(self.imageWidth * (left_percent / 100))] = 0


        y_scan_evade_obstacle = self.imageHeight - int(self.imageHeight * (bottom_percentage_evade_obstacle_scan / 100)) 
        x_positions_evade_obstacle = np.where(self.yellow_mask_to_evade_obstacle[y_scan_evade_obstacle] == 255)[0]
        if len(x_positions_evade_obstacle)> 0:
            self.get_logger().error("Line found: True")
            return True
        else:
            self.get_logger().info("Line found: False")
            return False

        



    def get_valid_range(self, index):
        if self.laser_data and 0 <= index < len(self.laser_data):
            val = self.laser_data[index]
            return val if not math.isinf(val) and not math.isnan(val) else float('inf')
        return float('inf')

    def lineDetectionYellow(self):
        if (self.current_frame is None):
            return
        #resized_frame = cv.resize(self.current_frame, None, fx=0.75, fy=0.75, interpolation=cv.INTER_CUBIC)
        resized_frame = cv.resize(self.current_frame, (800, 600))  # Set your desired width and height
        height, width = resized_frame.shape[:2]

        self.imageWidth = width
        self.imageHeight = height

        

        hsv_image = cv.cvtColor(resized_frame, cv.COLOR_BGR2HSV)

        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([95, 255, 255])

        yellow_mask = cv.inRange(hsv_image, lower_yellow, upper_yellow)

        self.yellow_mask_to_evade_obstacle = yellow_mask.copy()
        
        #new way of detecting point on line ---yellow below
        

        # Black out the left 10% of the image
        yellow_mask[:, :width // 10] = 0


        right_percent = 50
        yellow_mask[:, int(width * (1 - right_percent / 100)):] = 0

        

        
        # Pick the main scanline from bottom
        height = yellow_mask.shape[0]
        bottom_percentage_main_scan = 2.5
        if self.rightTurnCount == 5:
            if self.error == None:
                bottom_percentage_main_scan = 10.0
            else:
                bottom_percentage_main_scan = 2.5
        y_scan = height - int(height * (bottom_percentage_main_scan / 100))  

        


        #black out left for turn detection
        yellow_turn_blackout_ratio = 0.27
        yellow_mask_for_turn = yellow_mask.copy()
        yellow_mask_for_turn[:, :int(width * yellow_turn_blackout_ratio)] = 0


        #scan line for turn detection
        bottom_percentage = 20
        y_scan_turn_detection = height - int(height * (bottom_percentage / 100)) 
        x_positions_turn = np.where(yellow_mask_for_turn[y_scan_turn_detection] == 255)[0]
        
        if len(x_positions_turn) > 0:
            x_left = int(np.min(x_positions_turn))
            x_right = int(np.max(x_positions_turn))
            turn_yellow_width = x_right-x_left

            #self.get_logger().info(f"turn_yellow_width:{turn_yellow_width}")
            
        
            if 27 < turn_yellow_width < 32:
                self.turnDetected = False
            else:
                self.turnDetected = True
        else:
            self.turnDetected = True

        #scan line for tunnel exit
        # Copy the original mask
        yellow_mask_for_exit = yellow_mask_for_turn.copy()

        # --- Step 1: Morphological Filtering ---
        kernel = np.ones((5, 5), np.uint8)

        # Open to remove small dots/noise
        yellow_mask_for_exit = cv.morphologyEx(yellow_mask_for_exit, cv.MORPH_OPEN, kernel)

        # Close to fill small gaps in the line
        yellow_mask_for_exit = cv.morphologyEx(yellow_mask_for_exit, cv.MORPH_CLOSE, kernel)

        # --- Step 2 (Optional but Recommended): Keep only largest connected contour ---
        contours, _ = cv.findContours(yellow_mask_for_exit, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        clean_mask = np.zeros_like(yellow_mask_for_exit)

        if contours:
            largest_contour = max(contours, key=cv.contourArea)
            if cv.contourArea(largest_contour) > 500:  # adjust as needed
                cv.drawContours(clean_mask, [largest_contour], -1, 255, thickness=cv.FILLED)

        yellow_mask_for_exit = clean_mask  # Update with cleaned version
        bottom_percentage1 = 37
        y_scan_turn_detection1 = height - int(height * (bottom_percentage1 / 100)) 
        x_positions_turn1 = np.where(yellow_mask_for_exit[y_scan_turn_detection1] == 255)[0]
        
        if len(x_positions_turn1) > 0:
            x_left = int(np.min(x_positions_turn1))  # picks left most yellow 
            #self.get_logger().info(f"X_simple:{x_mean}")
            whiteLineFromCentreDistance3 = x_left  - width//2
            calib = -48
            self.get_logger().info(f"yellowLineFromCentreDistanceToExit tunnel:{whiteLineFromCentreDistance3}") #calibration
            self.errorExit = whiteLineFromCentreDistance3 - calib
            
        else:
            self.errorExit = None
        #self.get_logger().info(f"errorExit:{self.errorExit}")

        


        yellow_mask_bgr = cv.cvtColor(yellow_mask, cv.COLOR_GRAY2BGR)
        cv.line(yellow_mask_bgr, (0, y_scan), (width, y_scan), (255, 0, 0), 2)

        x_positions = np.where(yellow_mask[y_scan] == 255)[0]

        if len(x_positions) > 0:
            x_mean = int(np.min(x_positions))  # picks left most yellow (variable name is misleading)
            #self.get_logger().info(f"X_simple:{x_mean}")
            whiteLineFromCentreDistance2 = x_mean  - width//2
            calib = -265
            #self.get_logger().info(f"yellowLineFromCentreDistance:{whiteLineFromCentreDistance2}") #calibration
            self.error = whiteLineFromCentreDistance2 - calib  #293 is whiteLineFromCentreDistance when robot doesn't start to move. 329 for 20 percent from bottom
            #self.get_logger().info(f"error:{self.error}")
            yellow_mask_bgr = cv.cvtColor(yellow_mask, cv.COLOR_GRAY2BGR)
            cv.line(yellow_mask_bgr, (calib + width//2, 0), (calib + width//2, height), (255, 0, 0), 2) #vertical line
            cv.line(yellow_mask_bgr, (0, y_scan), (width, y_scan), (255, 0, 0), 2)#horizontal line
            cv.circle(yellow_mask_bgr, (x_mean, y_scan), 5, (0, 255, 0), 7) #main circle

            cv.line(yellow_mask_bgr, (0, y_scan_turn_detection), (width, y_scan_turn_detection), (0, 255, 0), 2)#horizontal line for turn detection
            cv.line(yellow_mask_bgr, (int(width * yellow_turn_blackout_ratio), 0), (int(width * yellow_turn_blackout_ratio), height), (0, 255, 0), 2) #vertical line of turn - left of which is blacked out
            cv.line(yellow_mask_bgr, (int(width * right_percent/100), 0), (int(width * right_percent/100), height), (0, 255, 0), 2)
            if len(x_positions_turn) > 0:
                cv.circle(yellow_mask_bgr, (x_left, y_scan_turn_detection), 5, (0, 255, 0), 7) #left turn circle
                cv.circle(yellow_mask_bgr, (x_right, y_scan_turn_detection), 5, (0, 255, 0), 7) #right turn circle
        else:
            self.error = None

        # Display
        
        
        cv.imshow("rotation control_yellow", yellow_mask_bgr)
        cv.waitKey(1)

    def image_callback(self, msg):
        try:
            cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        if self.signDetected == False:
            self.signDetected = self.detect_sign(cv_image)
            if self.signDetected:
                self.get_logger().info(f'Triangle sign detected: {self.signDetected}')
    
    def detect_sign(self, image):
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        #blank out left and right areas of image 
        height, width = hsv.shape[:2]
        right_percent = 50
        hsv[:, int(width * (1 - right_percent / 100)):] = 0

        left_percent = 0.27
        hsv[:, :int(width * left_percent)] = 0

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])

        red_mask = cv.bitwise_or(
            cv.inRange(hsv, lower_red1, upper_red1),
            cv.inRange(hsv, lower_red2, upper_red2)
        )
        yellow_mask = cv.inRange(hsv, lower_yellow, upper_yellow)

        

        contours_red, _ = cv.findContours(red_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours_yellow, _ = cv.findContours(yellow_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        debug_message = f"Red contours: {len(contours_red)}, Yellow contours: {len(contours_yellow)}."

        if not contours_red or not contours_yellow:
            return False

        for rc in contours_red:
            if len(cv.approxPolyDP(rc, 0.04 * cv.arcLength(rc, True), True)) != 3:
                continue

            for yc in contours_yellow:
                M = cv.moments(yc)
                if M["m00"] == 0:
                    continue
                cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                if cv.pointPolygonTest(rc, (cx, cy), False) >= 0:
                    return True

        return False


    def handBreak(self):
        self.get_logger().info("applying brake...")
        msg = Twist()
        msg.linear.x =0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def camera_callback(self, data):
        self.current_frame = self.br.imgmsg_to_cv2(data)

    def scanCallBack(self,msg):
        self.scanData = msg
        self.laser_data = msg.ranges

    def obstacleDetector(self):

        if self.scanData is None or not self.scanData.ranges:
            self.get_logger().warn("No valid LaserScan data received yet.")
            return

        # self.frontDistance = self.scanData.ranges[0]
        
        right_ranges = self.scanData.ranges[328:332]

        for i in range(len(right_ranges)):
            if right_ranges[i] == 0.0:
                right_ranges[i] = 10.0

        self.rightDistance = min(right_ranges)
        #self.get_logger().error(f"right distance:{self.rightDistance}")
        #self.get_logger().info(f"right obstscle detected:{self.rightObstacleDetected}")
        if self.rightDistance <= 0.54:
            if self.rightObstacleDetected == False:
                #self.get_logger().error("right obstacle detected!! setting it to true")
                self.rightObstacleDetected = True
        else:
            self.rightObstacleDetected = False

        #self.get_logger().info(f"rightDistance:{self.rightDistance}")

        # for i, distance in enumerate(self.scanData.ranges):
        #     self.get_logger().info(f"Range[{i}]: {distance}")


        front_ranges = self.scanData.ranges[-2:] + self.scanData.ranges[:2]
        front_ranges_pringle = self.scanData.ranges[-25:] + self.scanData.ranges[:25]

        # Replace 0.0 with 10.0
        for i in range(len(front_ranges_pringle)):
            if front_ranges_pringle[i] == 0.0:
                front_ranges_pringle[i] = 10.0

        # Replace 0.0 with 10.0
        for i in range(len(front_ranges)):
            if front_ranges[i] == 0.0:
                front_ranges[i] = 10.0

        
        self.minDistance = min(front_ranges)
        self.minDistancePringle = min(front_ranges_pringle)
        #self.get_logger().info(f"minDistancePringle:{self.minDistancePringle}")
        
        #self.get_logger().info(f"minDistance:{self.minDistance}")

        if self.minDistance <= 0.2:
            if self.obstacleDetected == False:
                self.get_logger().error("obstacle detected!! setting it to true")
                self.obstacleDetected = True
        else:
            self.obstacleDetected = False

        if self.minDistancePringle <= 0.25:
            if self.pringleDetected == False:
                self.get_logger().info("pringleDetected!! setting it to true")
                self.pringleDetected = True
        else:
            #self.get_logger().info("pringle lost!! setting it to false")
            self.pringleDetected = False
                
        

    def rotationController(self, error):
        # output needs to be passed to angular.z with negative sign
        now = self.get_clock().now()
        if self.prev_time is None:
            dt = 1e-6
        else:
            dt = (now - self.prev_time).nanoseconds / 1e9
            if dt == 0:
                dt = 1e-6

        # PID math
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        if self.turnDetected:
            output = (self.Kp_high * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        else:
            output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        
        # Update state
        self.prev_error = error
        self.prev_time = now

        return output


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = LineFollower()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
