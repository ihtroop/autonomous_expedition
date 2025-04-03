# from math import atan2
# import math
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan, Imu
# from std_msgs.msg import String, Float32
# from nav_msgs.msg import  Odometry
# from geometry_msgs.msg import Twist
# import time

# class ObstacleAvoider(Node):
    
#     def __init__(self):
#         super().__init__('obstacle_avoider')
        
#         self.scan_subscriber = self.create_subscription(String, '/obstacle_direction', self.scan_callback, 10)
#         self.pit_subscriber = self.create_subscription(String, '/pit_direction', self.pit_callback, 10)

#         self.put = self.create_subscription(Float32, 'obstacle_yaw', self.imu_callback, 10)
#         self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
#         self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
#         self.reset_state()
#         self.cmd_vel_msg = Twist()
        
#         self.initial_y = None
#         self.pit_direction = None

#         self.create_timer(0.1, self.avoid_obstacle)
    
#     def reset_state(self):
#         self.left_obstacle = False
#         self.front_obstacle = False
#         self.right_obstacle = False
#         self.record_time_diagonal = None
#         self.rotated_to_diagonal = False
#         self.target_yaw = None
#         self.yaw = None
#         self.obstacle = False
#         self.record_time_parallel_path = None
#         self.parallel_moved = True
#         self.diagonal_moved = False
#         self.rotating_angle = None
#         self.parallel_rotated = False
#         self.rotated_to_retrace = False
#         self.retraced = False
#         self.initial_y = None
#         self.pit_obstacle = False
    
#     def front_obstacle_again_reset(self):
#         self.record_time_diagonal = None
#         self.rotated_to_diagonal = False
#         self.record_time_parallel_path = None
#         self.parallel_moved = True
#         self.diagonal_moved = False
#         self.rotating_angle = None
#         self.parallel_rotated = False
#         self.rotated_to_retrace = False
#         self.retraced = False
    
#     def average(self, lst):
#         return sum(lst) / len(lst) if lst else float('inf')

#     def go_forward(self):
#         self.cmd_vel_msg.linear.x = 1.0
#         self.cmd_vel_msg.angular.z = 0.0
#         self.cmd_vel_publisher.publish(self.cmd_vel_msg)
    
#     def turn_left(self):
#         self.cmd_vel_msg.linear.x = 0.0
#         self.cmd_vel_msg.angular.z = 0.5
#         self.cmd_vel_publisher.publish(self.cmd_vel_msg)
    
#     def turn_right(self):
#         self.cmd_vel_msg.linear.x = 0.0
#         self.cmd_vel_msg.angular.z = -0.5
#         self.cmd_vel_publisher.publish(self.cmd_vel_msg)
    
#     def turn_to_yaw(self, yaw):
#         if self.rotating_angle < 0 and not self.retraced:
#             self.turn_left()
#         elif self.rotating_angle > 0 and not self.retraced:
#             self.turn_right()
#         elif self.rotating_angle < 0 and self.retraced:
#             self.turn_right()
#         elif self.rotating_angle > 0 and self.retraced:
#             self.turn_left()
        

#     def imu_callback(self, msg):
       
#         self.yaw = msg.data
        
#     def odom_callback(self, msg):
#         self.current_y = msg.pose.pose.position.y
#         # Store the initial y position once at the start
#         if self.initial_y is None:
#             self.initial_y = self.current_y
#             self.get_logger().info(f"Initial Y: {self.initial_y}")
            
#     def pit_callback(self, msg):
#         self.pit_direction = msg.data
#         self.pit_obstacle = self.pit_direction in ["Front_Left", "Front_Right"]

#     def scan_callback(self, msg):

        
#         self.direction = msg.data
#         self.get_logger().info(f"direction: {self.direction}")
        
#         if self.direction == 'Front':
#             self.front_obstacle = False
#             self.left_obstacle = False
#             self.right_obstacle = False
#             self.obstacle = False
#         elif self.direction == 'Front_Left' or self.direction == 'Front_Right':
#             self.front_obstacle = True
#             self.left_obstacle = False
#             self.right_obstacle = False
#         elif self.direction == 'Left':
#             self.front_obstacle = False
#             self.left_obstacle = True
#             self.right_obstacle = False
#         elif self.direction == 'Right':
#             self.front_obstacle = False
#             self.left_obstacle = False
#             self.right_obstacle = True
        
#         if self.target_yaw is None:
#             self.target_yaw = self.yaw
    
#     def avoid_obstacle(self):
#         if self.front_obstacle or self.pit_obstacle:
            
#             self.get_logger().info("Front obstacle detected, deciding rotation direction")
            
    

#             if self.direction == "Front_Left" or self.pit_direction == "Front_Left":
#                 self.get_logger().info("Rotating right")
#                 self.turn_right()
#                 self.rotating_angle = -0.5
#             elif self.direction == "Front_Right" or self.pit_direction == "Front_Right":
#                 self.get_logger().info("Rotating left")
#                 self.turn_left()
#                 self.rotating_angle = 0.5
            
#             self.obstacle = True
#             if self.target_yaw is None:
#                 self.target_yaw = self.yaw
#             print(f"target yaw: {self.target_yaw}, yaw: {self.yaw}")

#         elif self.obstacle and not self.front_obstacle and not self.pit_obstacle:
#             if self.record_time_diagonal is None:
#                 self.record_time_diagonal = time.time()
#             if self.record_time_diagonal is not None and not self.rotated_to_diagonal:
#                 if time.time() - self.record_time_diagonal < 4.0:
#                     self.get_logger().info("Moving diagonally to avoid")
#                     self.go_forward()
#                 else: 
#                     self.get_logger().info("Diagonal moved")
#                     self.diagonal_moved = True
#                     self.rotated_to_diagonal = True
#             elif self.diagonal_moved:
#                 # print(f"yaw: {self.yaw}, target_yaw: {self.target_yaw}, change: {abs(self.yaw - self.target_yaw)}")
#                 if abs(self.yaw - self.target_yaw) > 0.3:
#                     self.get_logger().info("Rotating to parallel path")
#                     self.turn_to_yaw(self.target_yaw)
#                 else:
#                     self.get_logger().info("Rotated to parallel path")
#                     self.diagonal_moved = False
#                     self.parallel_rotated = True
#             elif self.parallel_rotated:
#                 if self.record_time_parallel_path is None:
#                     self.record_time_parallel_path = time.time()
#                 if time.time() - self.record_time_parallel_path < 3.0:
#                     self.get_logger().info("Moving forward on parallel path")
#                     self.go_forward()
#                 else:
#                     self.parallel_rotated = False
#                     self.parallel_moved = True
#             elif self.parallel_moved:
#                 self.get_logger().info("Turning to retrace")
#                 target_yaw = -15 if self.rotating_angle > 0.0 else 15
#                 yaw_diff = abs(self.target_yaw - self.yaw)
#                 print(f"yaw: {self.yaw}, target: {self.target_yaw}, change: {yaw_diff}, 30deg: {math.radians(30)}")
#                 if yaw_diff < 7.0:
#                     self.turn_to_yaw(target_yaw)
#                 else:
#                     self.parallel_moved = False
#                     self.rotated_to_retrace = True
#             elif self.rotated_to_retrace:
#                 if abs(self.current_y - self.initial_y) > 0:
#                     self.get_logger().info("Moving forward to intial y")
#                     self.go_forward()
#                 else:
#                     self.rotated_to_retrace = False
#                     self.retraced = True
#             elif self.retraced:
#                 # print(f"yaw diff: {yaw_diff}, target_yaw: {self.target_yaw}")
#                 if abs(self.yaw - self.target_yaw) > 5:
#                     self.get_logger().info("Retracing complete, rotating to initial direction")
#                     self.turn_to_yaw(self.target_yaw)
#                 else:
#                     self.get_logger().error('Resetting obstacle avoidance')
#                     self.retraced = False
#                     self.reset_state()
#         elif not self.obstacle:
#             self.get_logger().info("No obstacle detected, moving forward")
#             self.go_forward()
#         else:
#             self.get_logger().error('No obstacle detected, but obstacle flag is set')
#             self.reset_state()
            
# def main(args=None):
#     rclpy.init(args=args)
#     obstacle_avoidance = ObstacleAvoider()
#     rclpy.spin(obstacle_avoidance)
#     obstacle_avoidance.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time

class ObstacleAvoider(Node):
    
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        self.scan_subscriber = self.create_subscription(String, '/obstacle_direction', self.scan_callback, 10)
        self.pit_subscriber = self.create_subscription(String, '/pit_direction', self.pit_callback, 10)
        self.put = self.create_subscription(Float32, 'obstacle_yaw', self.imu_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.cmd_vel_msg = Twist()
       
        self.current_y = 0.0
        self.yaw = 0.0
        self.initial_y = None
        self.obstacle_detected_y = None
        self.obstacle_avoidance_yaw = None
        
        self.obstacle = False
        self.front_obstacle = False
        self.left_obstacle = False
        self.right_obstacle = False
        self.pit_obstacle = False
        self.pit_direction = None
        self.direction = None
        self.should_start_forward = False
    
        self.start_forward_time = None
        self.movement_state = "default" 
        
        self.create_timer(0.1, self.avoid_obstacle)
    
    def reset_state(self):
        self.obstacle = False
        self.front_obstacle = False
        self.left_obstacle = False
        self.right_obstacle = False
        self.pit_obstacle = False
        self.movement_state = "default"
        self.start_forward_time = None
        self.should_start_forward = False
    
    def go_forward(self):
        self.cmd_vel_msg.linear.x = 1.0
        self.cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.cmd_vel_msg)
    
    def stop(self):
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.cmd_vel_msg)
    
    def turn(self, angular_velocity):
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(self.cmd_vel_msg)

    def imu_callback(self, msg):
        self.yaw = msg.data
        
    def odom_callback(self, msg):
        self.current_y = msg.pose.pose.position.y
        if self.initial_y is None:
            self.initial_y = self.current_y
            
    def pit_callback(self, msg):
        self.pit_direction = msg.data
        self.pit_obstacle = self.pit_direction in ["Front_Left", "Front_Right"]

    def scan_callback(self, msg):
        self.direction = msg.data
        self.get_logger().info(f"Direction: {self.direction}, State: {self.movement_state}")
        
        if self.direction in ['Front_Left', 'Front_Right']:
            if not self.obstacle:
                # self.obstacle_detected_y = self.current_y
                self.obstacle_detected_y = self.current_y

                
                self.obstacle = True
                self.front_obstacle = True
                self.should_start_forward = False
        
        elif self.direction in ['Front', 'Left', 'Right']:
            self.front_obstacle = False
            if self.obstacle:
                
                if self.movement_state == "default":
                    self.movement_state = "FORWARD_AFTER_AVOID"
                    self.start_forward_time = time.time()
                    self.obstacle_avoidance_yaw = self.yaw
                    self.get_logger().info("Starting return sequence - Forward movement")
            else:
                self.should_start_forward = True
                self.start_forward_time = time.time()
                self.get_logger().info("Starting forward movement for 3 seconds")
    
    def avoid_obstacle(self):
        if self.front_obstacle or self.pit_obstacle:
            self.get_logger().info("Avoiding obstacle")
            if self.direction == "Front_Left" or self.pit_direction == "Front_Left":

                self.turn(-0.5)  
            elif self.direction == "Front_Right" or self.pit_direction == "Front_Right":
                self.turn(0.5)   
                
        elif self.should_start_forward:
            if time.time() - self.start_forward_time <= 3.0:
                self.get_logger().info("Moving forward")
                self.go_forward()
            else:
                self.get_logger().info("Forward movement complete")
                self.should_start_forward = False
                self.reset_state()
                
        elif self.movement_state == "FORWARD_AFTER_AVOID":
            if time.time() - self.start_forward_time <= 4.0:
                self.go_forward()
            else:
                self.movement_state = "FIRST_TURN"
                self.get_logger().info("Starting first turn to original orientation")
                
        elif self.movement_state == "FIRST_TURN":
            angular_vel = 0.5 if self.obstacle_avoidance_yaw > 0 else -0.5
            if abs(self.yaw) > 5: 
                self.turn(angular_vel)
            else:
                self.movement_state = "SECOND_FORWARD"
                self.start_forward_time = time.time()
                self.get_logger().info("Starting second forward movement")
                
        elif self.movement_state == "SECOND_FORWARD":
            if time.time() - self.start_forward_time <= 4.0:
                self.go_forward()
            else:
                self.movement_state = "SECOND_TURN"
                self.get_logger().info(f"Starting second turn , {self.obstacle_avoidance_yaw}")
                
        elif self.movement_state == "SECOND_TURN":
            angular_vel = 0.5 if self.obstacle_avoidance_yaw > 0 else -0.5
            target_yaw = -self.obstacle_avoidance_yaw
            if abs(self.yaw - target_yaw) > 5:  
                self.turn(angular_vel)
            else:
                self.movement_state = "RETURN_TO_Y"
                self.get_logger().info("Moving to original Y position")
                
        elif self.movement_state == "RETURN_TO_Y":
            if abs(self.current_y - self.obstacle_detected_y) > 0.1:
                self.go_forward()
            else:
                self.movement_state = "FINAL_TURN"
                self.get_logger().info("Starting final turn to zero orientation")
                
        elif self.movement_state == "FINAL_TURN":
            angular_vel = 0.5 if self.obstacle_avoidance_yaw < 0 else -0.5
            target_yaw_final = 0.0
            if abs(self.yaw - target_yaw_final) > 5:
                self.turn(angular_vel)
            else:
                self.get_logger().info("Return sequence complete")
                self.reset_state()
                
        elif self.movement_state == "default" and not self.obstacle:
            self.go_forward()

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoider()
    rclpy.spin(obstacle_avoidance)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()