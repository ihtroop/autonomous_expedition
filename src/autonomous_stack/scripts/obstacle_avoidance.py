#!/usr/bin/env python3
# from math import atan2
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan, Imu 
# from std_msgs.msg import String
# from geometry_msgs.msg import Twist
# import time

# class ObstacleAvoider(Node):
#     def __init__(self):
#         super().__init__('obstacle_avoider')
        
#         self.reset_state()
#         self.cmd_vel_msg = Twist()
#         self.linear_velocity = 0.0
#         self.angular_velocity = 0.0
#         self.direction1 = None
#         self.direction2 = None
        
#         # Changed subscription callback to obstacle_callback instead of avoid_obstacle
#         self.scan_subscriber = self.create_subscription(
#             String, '/obstacle_direction', self.obstacle_callback, 10)
#         self.pit_subscriber = self.create_subscription(
#             String, '/pit_direction', self.pit_callback, 10)
#         self.imu = self.create_subscription(
#             Imu, 'imu', self.imu_callback, 10)
#         self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
#         # Timer for movement control
#         self.create_timer(0.2, self.control_loop)
        
#     def reset_state(self):
#         """Reset variables for handling new obstacles and pits."""
#         self.left_obstacle = False
#         self.front_obstacle = False
#         self.right_obstacle = False
#         self.left_pit = False
#         self.front_pit = False
#         self.right_pit = False
#         self.record_time_diagonal = None
#         self.target_yaw = None
#         self.yaw = None
#         self.obstacle = False
#         self.pit = False
#         self.prev_direction = "Front"
#         self.record_time_parallel_path = None
        
#     def imu_callback(self, msg):
#         q = msg.orientation
#         siny_cosp = 2 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
#         self.yaw = atan2(siny_cosp, cosy_cosp)

#     def pit_callback(self, msg):
#         self.direction2 = msg.data
#         if self.direction2 == 'Empty':
#             self.front_pit = False
#             self.left_pit = False
#             self.right_pit = False
#             self.pit = False
#         elif self.direction2 == 'Front_Left' or self.direction2 == 'Front_Right':
#             self.front_pit = True
#             self.left_pit = False
#             self.right_pit = False
              
#         if self.front_pit and self.target_yaw is None:
#             self.target_yaw = self.yaw
#             self.record_time_diagonal = time.time()

#     # Renamed from scan_callback to obstacle_callback to match its purpose
#     def obstacle_callback(self, msg):
#         self.direction1 = msg.data
#         if self.direction1 == 'Front':
#             self.front_obstacle = False
#             self.left_obstacle = False
#             self.right_obstacle = False
#             self.obstacle = False
#         elif self.direction1 == 'Front_Left' or self.direction1 == 'Front_Right':
#             self.front_obstacle = True
#             self.left_obstacle = False
#             self.right_obstacle = False
#         elif self.direction1 == 'Left':
#             self.front_obstacle = False
#             self.left_obstacle = True
#             self.right_obstacle = False
#         elif self.direction1 == 'Right':
#             self.front_obstacle = False
#             self.left_obstacle = False
#             self.right_obstacle = True
        
#         if self.front_obstacle and self.target_yaw is None:
#             self.target_yaw = self.yaw
#             self.record_time_diagonal = time.time()

#     # Renamed from avoid_obstacle to control_loop as it's now the timer callback
#     def control_loop(self):
#         if not self.front_obstacle and not self.obstacle and not self.front_pit and not self.pit:
#             self.get_logger().info("no obstacle and going front")
#             self.linear_velocity = 0.5
#             self.angular_velocity = 0.0
#             self.prev_direction = "Front"
#         elif self.front_obstacle or self.front_pit:
#             self.get_logger().info("Front obstacle detected, deciding rotation direction")
#             if self.direction1 == 'Front_Left' or self.direction2 == 'Front_Left':
#                 if self.prev_direction == "Front_Right":
#                     self.get_logger().info("Rotating left (more obstacle on the right)")
#                     self.linear_velocity = 0.0
#                     self.angular_velocity = 0.5
#                     self.rotating_angle = self.angular_velocity
#                     self.prev_direction = "Front_Right"
#                 else:
#                     self.get_logger().info("Rotating right (more obstacle on the left)")
#                     self.linear_velocity = 0.0
#                     self.angular_velocity = -0.5
#                     self.rotating_angle = self.angular_velocity
#                     self.prev_direction = "Front_Left"
#             elif self.direction1 == 'Front_Right' or self.direction2 == 'Front_Right':
#                 if self.prev_direction == "Front_Left":
#                     self.get_logger().info("Rotating right (more obstacle on the left)")
#                     self.linear_velocity = 0.0
#                     self.angular_velocity = -0.5
#                     self.rotating_angle = self.angular_velocity
#                     self.prev_direction = "Front_Left"
#                 else:
#                     self.get_logger().info("Rotating left (more obstacle on the right)")
#                     self.linear_velocity = 0.0
#                     self.angular_velocity = 0.5
#                     self.rotating_angle = self.angular_velocity
#                     self.prev_direction = "Front_Right" 
#             self.obstacle = True
#             if self.target_yaw is None:
#                 self.target_yaw = self.yaw
#                 self.record_time_diagonal = time.time()
#         elif self.obstacle and not self.front_obstacle:
#             if self.record_time_diagonal is not None and time.time() - self.record_time_diagonal < 2.0:
#                 self.get_logger().info("obstacle detected and moving diagonally")
#                 self.linear_velocity = 0.5
#                 self.angular_velocity = 0.0
#             elif abs(self.yaw - self.target_yaw) > 0.3:
#                 self.get_logger().info("rotating to parallel path")
#                 self.record_time_parallel_path = time.time()
#                 self.linear_velocity = 0.0
#                 self.angular_velocity = -self.rotating_angle
#             elif self.record_time_parallel_path is not None and time.time() - self.record_time_parallel_path < 2.0:
#                 self.get_logger().info("moving in parallel path")
#                 self.linear_velocity = 0.5
#                 self.angular_velocity = 0.0
#             else:
#                 self.get_logger().error('Resetting obstacle avoidance')
#                 self.reset_state()
#         elif self.left_obstacle or self.right_obstacle:
#             self.get_logger().info("left or right obstacle detected and moving forward")
#             self.linear_velocity = 0.5
#             self.angular_velocity = 0.0

#         self.cmd_vel_msg.linear.x = self.linear_velocity
#         self.cmd_vel_msg.angular.z = self.angular_velocity
#         self.cmd_vel_publisher.publish(self.cmd_vel_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     obstacle_avoider = ObstacleAvoider()
#     rclpy.spin(obstacle_avoider)
#     obstacle_avoider.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Int32
# from geometry_msgs.msg import Twist
# import time

# class ObstacleAvoider(Node):
#     def __init__(self):
#         super().__init__('obstacle_avoider')
        
#         self.reset_state()
#         self.cmd_vel_msg = Twist()
#         self.direction = None
        
#         # Navigation command flags
#         self.direction_command = False
#         self.distance_command = False
#         self.turn_command = False
        
#         # Subscribe to obstacle direction
#         self.scan_subscriber = self.create_subscription(
#             String, 
#             '/obstacle_direction', 
#             self.scan_callback, 
#             10
#         )
        
#         # Subscribe to navigation commands
#         self.direction_subscriber = self.create_subscription(
#             String,
#             'direction',
#             self.direction_command_callback,
#             10
#         )
        
#         self.distance_subscriber = self.create_subscription(
#             Int32,
#             'distance',
#             self.distance_command_callback,
#             10
#         )
        
#         self.turn_subscriber = self.create_subscription(
#             Int32,
#             'turn',
#             self.turn_command_callback,
#             10
#         )
        
#         # Publisher for velocity commands
#         self.cmd_vel_publisher = self.create_publisher(
#             Twist, 
#             'cmd_vel', 
#             10
#         )

#         self.create_timer(0.2, self.avoid_obstacle)
        
#     def reset_state(self):
#         self.left_obstacle = False
#         self.front_obstacle = False
#         self.right_obstacle = False
#         self.record_time_diagonal = None
#         self.record_time_parallel_path = None
#         self.obstacle = False
#         self.rotating_angle = 0.0
    
#     def direction_command_callback(self, msg):
#         self.direction_command = True
#         self.get_logger().info('Direction command received - pausing obstacle avoidance')
        
#     def distance_command_callback(self, msg):
#         self.distance_command = True
#         self.get_logger().info('Distance command received - pausing obstacle avoidance')
        
#     def turn_command_callback(self, msg):
#         self.turn_command = True
#         self.get_logger().info('Turn command received - pausing obstacle avoidance')
        
#     def scan_callback(self, msg):
#         self.direction = msg.data

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

#         if self.front_obstacle:
#             self.record_time_diagonal = time.time()

#     def is_navigation_in_progress(self):
#         # Check if any navigation command is active
#         if any([self.direction_command, self.distance_command, self.turn_command]):
#             return True
#         return False

#     def avoid_obstacle(self):
#         # Don't proceed if navigation commands are active
#         if self.is_navigation_in_progress():
#             return
            
#         # Reset navigation command flags for next cycle
#         self.direction_command = False
#         self.distance_command = False
#         self.turn_command = False
        
#         # Only proceed if front obstacle is detected
#         if not self.front_obstacle:
#             return

#         if self.direction == 'Front_Left':
#             self.get_logger().info("Rotating right to avoid left obstacle")
#             self.cmd_vel_msg.linear.x = 0.0
#             self.cmd_vel_msg.angular.z = -0.5
#             self.rotating_angle = self.cmd_vel_msg.angular.z
#             self.obstacle = True
            
#         elif self.direction == 'Front_Right':
#             self.get_logger().info("Rotating left to avoid right obstacle")
#             self.cmd_vel_msg.linear.x = 0.0
#             self.cmd_vel_msg.angular.z = 0.5
#             self.rotating_angle = self.cmd_vel_msg.angular.z
#             self.obstacle = True

#         # Only publish velocity commands during active avoidance and when no navigation is in progress
#         self.cmd_vel_publisher.publish(self.cmd_vel_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     obstacle_avoider = ObstacleAvoider()
#     rclpy.spin(obstacle_avoider)
#     obstacle_avoider.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Int32
# from geometry_msgs.msg import Twist
# import time

# class ObstacleAvoider(Node):
#     def __init__(self):
#         super().__init__('obstacle_avoider')
        
#         self.reset_state()
#         self.cmd_vel_msg = Twist()
#         self.direction = None
        
#         # Flag to track alignment sequence
#         self.alignment_active = False
        
#         # Navigation command flags
#         self.direction_command = False
#         self.distance_command = False
#         self.turn_command = False
        
#         # Subscribe to alignment
#         self.align_subscriber = self.create_subscription(
#             Twist, "/align_publisher", self.alignment_callback, 10
#         )
        
#         # Subscribe to obstacle direction
#         self.scan_subscriber = self.create_subscription(
#             String, 
#             '/obstacle_direction', 
#             self.scan_callback, 
#             10
#         )
        
#         # Subscribe to navigation commands
#         self.direction_subscriber = self.create_subscription(
#             String,
#             'direction',
#             self.direction_command_callback,
#             10
#         )
        
#         self.distance_subscriber = self.create_subscription(
#             Int32,
#             'distance',
#             self.distance_command_callback,
#             10
#         )
        
#         self.turn_subscriber = self.create_subscription(
#             Int32,
#             'turn',
#             self.turn_command_callback,
#             10
#         )
        
#         # Publisher for velocity commands
#         self.cmd_vel_publisher = self.create_publisher(
#             Twist, 
#             'cmd_vel', 
#             10
#         )

#         self.create_timer(0.2, self.avoid_obstacle)
        
#     def reset_state(self):
#         self.left_obstacle = False
#         self.front_obstacle = False
#         self.right_obstacle = False
#         self.record_time_diagonal = None
#         self.record_time_parallel_path = None
#         self.obstacle = False
#         self.rotating_angle = 0.0
#         self.alignment_active = False
    
#     def alignment_callback(self, msg):
#         self.alignment_active = True
#         # Clear any existing obstacle states when alignment starts
#         self.left_obstacle = False
#         self.front_obstacle = False
#         self.right_obstacle = False
#         self.obstacle = False
#         self.get_logger().info('Alignment received - disabling obstacle detection')
        
#     def direction_command_callback(self, msg):
#         self.direction_command = True
#         self.get_logger().info('Direction command received')
        
#     def distance_command_callback(self, msg):
#         self.distance_command = True
#         self.get_logger().info('Distance command received')
        
#     def turn_command_callback(self, msg):
#         self.turn_command = True
#         self.alignment_active = False  # Re-enable obstacle detection
#         self.get_logger().info('Turn command received - re-enabling obstacle detection')
        
#     def scan_callback(self, msg):
#         # If alignment is active, store the direction but don't set any obstacle flags
#         if self.alignment_active:
#             self.direction = msg.data  # Just store the direction
#             return
            
#         self.direction = msg.data

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

#         if self.front_obstacle:
#             self.record_time_diagonal = time.time()

#     def is_navigation_in_progress(self):
#         # Check if any navigation command is active
#         if any([self.direction_command, self.distance_command, self.turn_command]):
#             return True
#         return False

#     def avoid_obstacle(self):
#         # First check: if alignment is active, don't do any obstacle avoidance
#         if self.alignment_active:
#             # Reset the cmd_vel message to ensure no movement
#             self.cmd_vel_msg.linear.x = 0.0
#             self.cmd_vel_msg.angular.z = 0.0
#             return
            
#         # Second check: normal navigation commands
#         if self.is_navigation_in_progress():
#             return
            
#         # Reset navigation command flags for next cycle
#         self.direction_command = False
#         self.distance_command = False
#         self.turn_command = False
        
#         # Only proceed if front obstacle is detected
#         if not self.front_obstacle:
#             return

#         if self.direction == 'Front_Left':
#             self.get_logger().info("Rotating right to avoid left obstacle")
#             self.cmd_vel_msg.linear.x = 0.0
#             self.cmd_vel_msg.angular.z = -0.5
#             self.rotating_angle = self.cmd_vel_msg.angular.z
#             self.obstacle = True
            
#         elif self.direction == 'Front_Right':
#             self.get_logger().info("Rotating left to avoid right obstacle")
#             self.cmd_vel_msg.linear.x = 0.0
#             self.cmd_vel_msg.angular.z = 0.5
#             self.rotating_angle = self.cmd_vel_msg.angular.z
#             self.obstacle = True

#         # Only publish velocity commands if not in alignment sequence
#         if not self.alignment_active:
#             self.cmd_vel_publisher.publish(self.cmd_vel_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     obstacle_avoider = ObstacleAvoider()
#     rclpy.spin(obstacle_avoider)
#     obstacle_avoider.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# PERFECT OBSTACLE AVOIDANCE WITHOUT PIT

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Int32
# from geometry_msgs.msg import Twist
# import time

# class ObstacleAvoider(Node):
#     def __init__(self):
#         super().__init__('obstacle_avoider')
        
#         self.reset_state()
#         self.cmd_vel_msg = Twist()
#         self.direction = None
        
#         # Flag to track alignment sequence
#         self.alignment_active = False
        
#         # Navigation command flags
#         self.direction_command = False
#         self.distance_command = False
#         self.turn_command = False
        
#         # Subscribe to alignment
#         self.align_subscriber = self.create_subscription(
#             Twist, "/align_publisher", self.alignment_callback, 10
#         )
        
#         # Subscribe to obstacle direction
#         self.scan_subscriber = self.create_subscription(
#             String, 
#             '/obstacle_direction', 
#             self.scan_callback, 
#             10
#         )
        
#         # Subscribe to navigation commands
#         self.direction_subscriber = self.create_subscription(
#             String,
#             'direction',
#             self.direction_command_callback,
#             10
#         )
        
#         self.distance_subscriber = self.create_subscription(
#             Int32,
#             'distance',
#             self.distance_command_callback,
#             10
#         )
        
#         self.turn_subscriber = self.create_subscription(
#             Int32,
#             'turn',
#             self.turn_command_callback,
#             10
#         )
        
#         # Publisher for velocity commands
#         self.cmd_vel_publisher = self.create_publisher(
#             Twist, 
#             'cmd_vel', 
#             10
#         )

#         self.create_timer(0.2, self.avoid_obstacle)
        
#     def reset_state(self):
#         self.left_obstacle = False
#         self.front_obstacle = False
#         self.right_obstacle = False
#         self.record_time_diagonal = None
#         self.record_time_parallel_path = None
#         self.obstacle = False
#         self.rotating_angle = 0.0
#         self.alignment_active = False
    
#     def alignment_callback(self, msg):
#         self.alignment_active = True
#         # Clear any existing obstacle states when alignment starts
#         self.left_obstacle = False
#         self.front_obstacle = False
#         self.right_obstacle = False
#         self.obstacle = False
#         self.get_logger().info('Alignment received - disabling obstacle detection')
        
#     def direction_command_callback(self, msg):
#         self.direction_command = True
#         self.get_logger().info('Direction command received')
        
#     def distance_command_callback(self, msg):
#         self.distance_command = True
#         self.get_logger().info('Distance command received')
        
#     def turn_command_callback(self, msg):
#         self.alignment_active = False  # Re-enable obstacle detection
#         # Reset all navigation command flags to allow obstacle avoidance to resume
#         self.direction_command = False
#         self.distance_command = False
#         self.turn_command = False
#         self.get_logger().info('Turn command received - re-enabling obstacle detection')
        
#     def scan_callback(self, msg):
#         # If alignment is active, store the direction but don't set any obstacle flags
#         if self.alignment_active:
#             self.direction = msg.data  # Just store the direction
#             return
            
#         self.direction = msg.data

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

#         if self.front_obstacle:
#             self.record_time_diagonal = time.time()

#     def is_navigation_in_progress(self):
#         # Check if any navigation command is active
#         if any([self.direction_command, self.distance_command, self.turn_command]):
#             return True
#         return False

#     def avoid_obstacle(self):
#         # First check: if alignment is active, don't do any obstacle avoidance
#         if self.alignment_active:
#             return
            
#         # Second check: normal navigation commands
#         if self.is_navigation_in_progress():
#             return
            
#         # Only proceed if front obstacle is detected
#         if not self.front_obstacle:
#             return

#         if self.direction == 'Front_Left':
#             self.get_logger().info("Rotating right to avoid left obstacle")
#             self.cmd_vel_msg.linear.x = 0.0
#             self.cmd_vel_msg.angular.z = -0.5
#             self.rotating_angle = self.cmd_vel_msg.angular.z
#             self.obstacle = True
#             self.cmd_vel_publisher.publish(self.cmd_vel_msg)
            
#         elif self.direction == 'Front_Right':
#             self.get_logger().info("Rotating left to avoid right obstacle")
#             self.cmd_vel_msg.linear.x = 0.0
#             self.cmd_vel_msg.angular.z = 0.5
#             self.rotating_angle = self.cmd_vel_msg.angular.z
#             self.obstacle = True
#             self.cmd_vel_publisher.publish(self.cmd_vel_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     obstacle_avoider = ObstacleAvoider()
#     rclpy.spin(obstacle_avoider)
#     obstacle_avoider.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
import time

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        self.reset_state()
        self.cmd_vel_msg = Twist()
        self.direction = None
        self.direction2 = None
        
        # Flag to track alignment sequence
        self.alignment_active = False
        
        # Navigation command flags
        self.direction_command = False
        self.distance_command = False
        self.turn_command = False
        
        # Subscribe to alignment
        self.align_subscriber = self.create_subscription(
            Twist, "/align_publisher", self.alignment_callback, 10
        )
        
        # Subscribe to obstacle direction
        self.scan_subscriber = self.create_subscription(
            String, 
            '/obstacle_direction', 
            self.scan_callback, 
            10
        )
        
        self.pit_subscirber = self.create_subscription(
            String,
            '/pit_direction',
            self.pit_callback,
            10
        )

        # Subscribe to navigation commands
        self.direction_subscriber = self.create_subscription(
            String,
            'direction',
            self.direction_command_callback,
            10
        )
        
        self.distance_subscriber = self.create_subscription(
            Int32,
            'distance',
            self.distance_command_callback,
            10
        )
        
        self.turn_subscriber = self.create_subscription(
            Int32,
            'turn',
            self.turn_command_callback,
            10
        )
        
        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            'cmd_vel', 
            10
        )

        self.create_timer(0.2, self.avoid_obstacle)
        
    def reset_state(self):
        self.left_obstacle = False
        self.front_obstacle = False
        self.right_obstacle = False
        self.record_time_diagonal = None
        self.record_time_parallel_path = None
        self.obstacle = False
        self.rotating_angle = 0.0
        self.alignment_active = False
    
    def alignment_callback(self, msg):
        self.alignment_active = True
        # Clear any existing obstacle states when alignment starts
        self.left_obstacle = False
        self.front_obstacle = False
        self.right_obstacle = False
        self.obstacle = False
        self.get_logger().info('Alignment received - disabling obstacle detection')
        
    def direction_command_callback(self, msg):
        self.direction_command = True
        self.get_logger().info('Direction command received')
        
    def distance_command_callback(self, msg):
        self.distance_command = True
        self.get_logger().info('Distance command received')
        
    def turn_command_callback(self, msg):
        self.alignment_active = False  # Re-enable obstacle detection
        # Reset all navigation command flags to allow obstacle avoidance to resume
        self.direction_command = False
        self.distance_command = False
        self.turn_command = False
        self.get_logger().info('Turn command received - re-enabling obstacle detection')
        
    def scan_callback(self, msg):
        # If alignment is active, store the direction but don't set any obstacle flags
        if self.alignment_active:
            self.direction = msg.data  # Just store the direction
            return
            
        self.direction = msg.data

        if self.direction == 'Front':
            self.front_obstacle = False
            self.left_obstacle = False
            self.right_obstacle = False
            self.obstacle = False
        elif self.direction == 'Front_Left' or self.direction == 'Front_Right':
            self.front_obstacle = True
            self.left_obstacle = False
            self.right_obstacle = False
        elif self.direction == 'Left':
            self.front_obstacle = False
            self.left_obstacle = True
            self.right_obstacle = False
        elif self.direction == 'Right':
            self.front_obstacle = False
            self.left_obstacle = False
            self.right_obstacle = True

        if self.front_obstacle:
            self.record_time_diagonal = time.time()

    def pit_callback(self, msg):
        if self.alignment_active:
            self.direction2 = msg.data  # Just store the direction
            return
            
        self.direction2 = msg.data

        if self.direction2 == 'Front':
            self.front_obstacle = False
            self.left_obstacle = False
            self.right_obstacle = False
            self.obstacle = False
        elif self.direction2 == 'Front_Left' or self.direction2 == 'Front_Right':
            self.get_logger().info(f"{msg.data}")
            self.front_obstacle = True
            self.left_obstacle = False
            self.right_obstacle = False

        if self.front_obstacle:
            self.record_time_diagonal = time.time()

    def is_navigation_in_progress(self):
        # Check if any navigation command is active
        if any([self.direction_command, self.distance_command, self.turn_command]):
            return True
        return False

    def avoid_obstacle(self):
        # First check: if alignment is active, don't do any obstacle avoidance
        if self.alignment_active:
            return
            
        # Second check: normal navigation commands
        if self.is_navigation_in_progress():
            return
            
        # Only proceed if front obstacle is detected
        if not self.front_obstacle:
            return

        if self.direction == 'Front_Left' or self.direction2 == 'Front_Left':
            self.get_logger().info("Rotating right to avoid left obstacle")
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = -0.5
            self.rotating_angle = self.cmd_vel_msg.angular.z
            self.obstacle = True
            self.cmd_vel_publisher.publish(self.cmd_vel_msg)
            
        elif self.direction == 'Front_Right' or self.direction2 == 'Front_Right':
            self.get_logger().info("Rotating left to avoid right obstacle")
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.5
            self.rotating_angle = self.cmd_vel_msg.angular.z
            self.obstacle = True
            self.cmd_vel_publisher.publish(self.cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoider = ObstacleAvoider()
    rclpy.spin(obstacle_avoider)
    obstacle_avoider.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    