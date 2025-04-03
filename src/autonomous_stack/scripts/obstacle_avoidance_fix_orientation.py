
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Int32, Float32
# from geometry_msgs.msg import Twist
# import time
# import math

# class ObstacleAvoider(Node):
#     def __init__(self):
#         super().__init__('obstacle_avoider')
        
#         self.reset_state()
#         self.cmd_vel_msg = Twist()
        
#         # Yaw tracking
#         self.current_yaw = 0.0
#         self.yaw_subscriber = self.create_subscription(
#             Float32, 
#             '/obstacle_yaw', 
#             self.yaw_callback, 
#             10
#         )
        
#         # Flag to track alignment sequence
#         self.alignment_active = False
        
#         # Navigation command flags
#         self.direction_command = False
#         self.distance_command = False
#         self.turn_command = False
        
#         # Timing for linear motion
#         self.linear_motion_start_time = None
        
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
        
#         self.pit_subscriber = self.create_subscription(
#             String,
#             '/pit_direction',
#             self.pit_callback,
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
#         self.obstacle = False
#         self.rotating_angle = 0.0
#         self.alignment_active = False
#         self.avoidance_stage = 'nothing'
#         self.current_yaw = 0.0
#         self.waiting_for_next_stage = False
#         self.last_obstacle_direction = None
        
#     def yaw_callback(self, msg):
#         self.current_yaw = msg.data
        
#     def alignment_callback(self, msg):
#         self.alignment_active = True
#         self.reset_state()
#         self.get_logger().info('Alignment received - disabling obstacle detection')
        
#     def direction_command_callback(self, msg):
#         self.direction_command = True
#         self.get_logger().info('Direction command received')
        
#     def distance_command_callback(self, msg):
#         self.distance_command = True
#         self.get_logger().info('Distance command received')
        
#     def turn_command_callback(self, msg):
#         self.alignment_active = False
#         self.direction_command = False
#         self.distance_command = False
#         self.turn_command = False
#         self.get_logger().info('Turn command received - re-enabling obstacle detection')
        
#     def scan_callback(self, msg):
#         if self.alignment_active:
#             return
            
#         if msg.data in ['Front_Left', 'Front_Right']:
#             self.front_obstacle = True
#             self.last_obstacle_direction = msg.data
#             self.avoidance_stage = 'ANGULAR'
#             self.get_logger().info(f"Obstacle detected: {msg.data}")
#             self.waiting_for_next_stage = True
#         elif msg.data in ['Front', 'Left', 'Right'] and self.waiting_for_next_stage:
#             self.avoidance_stage = 'LINEAR'
#             self.waiting_for_next_stage = False
#             self.get_logger().info(f"Proceeding to linear stage: {msg.data}")

#     def pit_callback(self, msg):
#         if self.alignment_active:
#             return
            
#         if msg.data in ['Front_Left', 'Front_Right']:
#             self.front_obstacle = True
#             self.last_obstacle_direction = msg.data
#             self.avoidance_stage = 'ANGULAR'
#             self.get_logger().info(f"Pit detected: {msg.data}")
#             self.waiting_for_next_stage = True
#         elif msg.data in ['Front', 'Left', 'Right'] and self.waiting_for_next_stage:
#             self.avoidance_stage = 'LINEAR'
#             self.waiting_for_next_stage = False
#             self.get_logger().info(f"Proceeding to linear stage: {msg.data}")

#     def is_navigation_in_progress(self):
#         return any([self.direction_command, self.distance_command, self.turn_command])

#     def avoid_obstacle(self):
#         if self.alignment_active or self.is_navigation_in_progress():
#             return

#         current_time = time.time()

#         # Angular stage when Front_Left or Front_Right is detected
#         if self.avoidance_stage == 'ANGULAR':
#             if self.front_obstacle:
#                 self.get_logger().info("Performing angular motion")
#                 self.cmd_vel_msg.linear.x = 0.0
#                 # Rotate based on the last detected obstacle direction
#                 self.cmd_vel_msg.angular.z = -0.5 if 'Right' in str(self.last_obstacle_direction) else 0.5
#                 self.cmd_vel_publisher.publish(self.cmd_vel_msg)

#         # Linear motion stage 
#         elif self.avoidance_stage == 'LINEAR':
#             self.get_logger().info("Starting linear motion")
#             self.cmd_vel_msg.linear.x = 1.0
#             self.cmd_vel_msg.angular.z = 0.0
#             self.linear_motion_start_time = current_time
#             self.avoidance_stage = 'forward_wait'
#             self.cmd_vel_publisher.publish(self.cmd_vel_msg)

#         elif self.avoidance_stage == 'forward_wait':
#             if current_time - self.linear_motion_start_time >= 2.0:
#                 self.avoidance_stage = 'yaw_correct'

#         # Yaw correction stage
#         elif self.avoidance_stage == 'yaw_correct':
#             if abs(self.current_yaw) > 15:
#                 self.get_logger().info(f"Correcting yaw: {self.current_yaw}")
#                 self.cmd_vel_msg.linear.x = 0.0
#                 self.cmd_vel_msg.angular.z = -0.5 if self.current_yaw > 0 else 0.5
#                 self.cmd_vel_publisher.publish(self.cmd_vel_msg)
#             else:
#                 self.get_logger().info("Yaw corrected, resetting state")
#                 self.cmd_vel_msg.linear.x = 1.0
#                 self.cmd_vel_msg.angular.z = 0.0
#                 self.cmd_vel_publisher.publish(self.cmd_vel_msg)
#                 self.reset_state()

# def main(args=None):
#     rclpy.init(args=args)
#     obstacle_avoider = ObstacleAvoider()
#     rclpy.spin(obstacle_avoider)
#     obstacle_avoider.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#works properly without abstacle_active_thing
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Int32, Float32
# from geometry_msgs.msg import Twist
# import time
# import math

# class ObstacleAvoider(Node):
#     def __init__(self):
#         super().__init__('obstacle_avoider')
        
#         self.reset_state()
#         self.cmd_vel_msg = Twist()
        
#         # Yaw tracking
#         self.current_yaw = 0.0
#         self.yaw_subscriber = self.create_subscription(
#             Float32, 
#             '/obstacle_yaw', 
#             self.yaw_callback, 
#             10
#         )
        
#         # Flag to track alignment sequence
#         self.alignment_active = False
        
#         # Navigation command flags
#         self.direction_command = False
#         self.distance_command = False
#         self.turn_command = False
        
#         # Timing for linear motion
#         self.linear_motion_start_time = None
        
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
        
#         self.pit_subscriber = self.create_subscription(
#             String,
#             '/pit_direction',
#             self.pit_callback,
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
#         self.obstacle = False
#         self.rotating_angle = 0.0
#         self.alignment_active = False
#         self.avoidance_stage = 'nothing'
#         self.current_yaw = 0.0
#         self.waiting_for_next_stage = False
#         self.last_obstacle_direction = None
        
#     def yaw_callback(self, msg):
#         self.current_yaw = msg.data
        
#     def alignment_callback(self, msg):
#         # Completely disable obstacle detection during alignment
#         self.alignment_active = True
#         #self.reset_state()
#         self.left_obstacle = False
#         self.front_obstacle = False
#         self.right_obstacle = False
#         self.obstacle = False
#         self.get_logger().info('Alignment received - FULLY disabling obstacle detection')
        
#     def direction_command_callback(self, msg):
#         if self.alignment_active:
#             return
#         self.direction_command = True
#         self.get_logger().info('Direction command received')
        
#     def distance_command_callback(self, msg):
#         if self.alignment_active:
#             return
#         self.distance_command = True
#         self.get_logger().info('Distance command received')
        
#     def turn_command_callback(self, msg):
#         # Re-enable obstacle avoidance only when turn is complete
#         self.alignment_active = False
#         self.direction_command = False
#         self.distance_command = False
#         self.turn_command = False
#         #self.reset_state()  # Reset to initial state
#         self.get_logger().info('Turn command received - re-enabling obstacle detection')
        
#     def scan_callback(self, msg):
#         # Completely ignore scan if in alignment mode
#         if self.alignment_active:
#             return
            
#         if msg.data in ['Front_Left', 'Front_Right']:
#             self.front_obstacle = True
#             self.last_obstacle_direction = msg.data
#             self.avoidance_stage = 'ANGULAR'
#             self.get_logger().info(f"Obstacle detected: {msg.data}")
#             self.waiting_for_next_stage = True
#         elif msg.data in ['Front', 'Left', 'Right'] and self.waiting_for_next_stage:
#             self.avoidance_stage = 'LINEAR'
#             self.waiting_for_next_stage = False
#             self.get_logger().info(f"Proceeding to linear stage: {msg.data}")

#     def pit_callback(self, msg):
#         # Completely ignore pit detection if in alignment mode
#         if self.alignment_active:
#             return
            
#         if msg.data in ['Front_Left', 'Front_Right']:
#             self.front_obstacle = True
#             self.last_obstacle_direction = msg.data
#             self.avoidance_stage = 'ANGULAR'
#             self.get_logger().info(f"Pit detected: {msg.data}")
#             self.waiting_for_next_stage = True
#         elif msg.data in ['Front', 'Left', 'Right'] and self.waiting_for_next_stage:
#             self.avoidance_stage = 'LINEAR'
#             self.waiting_for_next_stage = False
#             self.get_logger().info(f"Proceeding to linear stage: {msg.data}")

#     def is_navigation_in_progress(self):
#         #return any([self.direction_command, self.distance_command, self.turn_command])
#         if any([self.direction_command, self.distance_command, self.turn_command]):
#             return True
#         return False

#     def avoid_obstacle(self):
#         # Completely disable during alignment or ongoing navigation commands
#         if self.alignment_active or self.is_navigation_in_progress():
#             return

#         current_time = time.time()

#         # Angular stage when Front_Left or Front_Right is detected
#         if self.avoidance_stage == 'ANGULAR':
#             if self.front_obstacle:
#                 self.get_logger().info("Performing angular motion")
#                 self.cmd_vel_msg.linear.x = 0.0
#                 # Rotate based on the last detected obstacle direction
#                 self.cmd_vel_msg.angular.z = -0.5 if 'Right' in str(self.last_obstacle_direction) else 0.5
#                 self.cmd_vel_publisher.publish(self.cmd_vel_msg)

#         # Linear motion stage 
#         elif self.avoidance_stage == 'LINEAR':
#             self.get_logger().info("Starting linear motion")
#             self.cmd_vel_msg.linear.x = 1.0
#             self.cmd_vel_msg.angular.z = 0.0
#             self.linear_motion_start_time = current_time
#             self.avoidance_stage = 'forward_wait'
#             self.cmd_vel_publisher.publish(self.cmd_vel_msg)

#         elif self.avoidance_stage == 'forward_wait':
#             if current_time - self.linear_motion_start_time >= 2.0:
#                 self.avoidance_stage = 'yaw_correct'

#         # Yaw correction stage
#         elif self.avoidance_stage == 'yaw_correct':
#             if abs(self.current_yaw) > 5:
#                 self.get_logger().info(f"Correcting yaw: {self.current_yaw}")
#                 self.cmd_vel_msg.linear.x = 0.0
#                 self.cmd_vel_msg.angular.z = -0.5 if self.current_yaw > 0 else 0.5
#                 self.cmd_vel_publisher.publish(self.cmd_vel_msg)
#             else:
#                 self.get_logger().info("Yaw corrected, resetting state")
#                 self.cmd_vel_msg.linear.x = 0.0
#                 self.cmd_vel_msg.angular.z = 0.0
#                 self.cmd_vel_publisher.publish(self.cmd_vel_msg)
#                 self.reset_state()

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
from std_msgs.msg import String, Int32, Float32
from geometry_msgs.msg import Twist
import time
import math

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        self.reset_state()
        self.cmd_vel_msg = Twist()
        
        # Yaw tracking
        self.current_yaw = 0.0
        self.yaw_subscriber = self.create_subscription(
            Float32, 
            '/obstacle_yaw', 
            self.yaw_callback, 
            10
        )
        
        # Flag to track alignment sequence
        self.alignment_active = False
        
        # Navigation command flags
        self.direction_command = False
        self.distance_command = False
        self.turn_command = False
        
        # Timing for linear motion
        self.linear_motion_start_time = None
        
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
        
        self.pit_subscriber = self.create_subscription(
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
            '/cmd_vel', 
            10
        )

        self.obstacle_avoidance_status_publisher = self.create_publisher(
            String, 
            '/obstacle_avoidance_status', 
            10
        )
        
        self.create_timer(0.2, self.avoid_obstacle)
        
    def reset_state(self):
        self.left_obstacle = False
        self.front_obstacle = False
        self.right_obstacle = False
        self.obstacle = False
        self.rotating_angle = 0.0
        self.alignment_active = False
        self.avoidance_stage = 'nothing'
        self.current_yaw = 0.0
        self.waiting_for_next_stage = False
        self.last_obstacle_direction = None
        self.count=0
        
    def yaw_callback(self, msg):
        self.current_yaw = msg.data
        
    def alignment_callback(self, msg):
  
        self.alignment_active = True
     
        status_msg = String()
        status_msg.data = 'disable'
        self.obstacle_avoidance_status_publisher.publish(status_msg)
        
        self.left_obstacle = False
        self.front_obstacle = False
        self.right_obstacle = False
        self.obstacle = False
        self.get_logger().info('Alignment received - FULLY disabling obstacle detection')
        
    def direction_command_callback(self, msg):
        if self.alignment_active:
            return
        self.direction_command = True
        self.get_logger().info('Direction command received')
        
    def distance_command_callback(self, msg):
        if self.alignment_active:
            return
        self.distance_command = True
        self.get_logger().info('Distance command received')
        
    def turn_command_callback(self, msg):
        self.alignment_active = False
        self.direction_command = False
        self.distance_command = False
        self.turn_command = False
        
        # Publish ACTIVE status when turn is complete
        status_msg = String()
        status_msg.data = 'enable'
        self.obstacle_avoidance_status_publisher.publish(status_msg)
        
        self.get_logger().info('Turn command received - re-enabling obstacle detection')
        
    def scan_callback(self, msg):
        # Completely ignore scan if in alignment mode
        if self.alignment_active:
            return
            
        if msg.data in ['Front_Left', 'Front_Right']:
            self.front_obstacle = True
            self.last_obstacle_direction = msg.data
            self.avoidance_stage = 'ANGULAR'
            self.get_logger().info(f"Obstacle detected: {msg.data}")
            self.waiting_for_next_stage = True
        elif msg.data in ['Front', 'Left', 'Right'] and self.waiting_for_next_stage:
            self.avoidance_stage = 'LINEAR'
            self.waiting_for_next_stage = False
            self.get_logger().info(f"Proceeding to linear stage: {msg.data}")

    def pit_callback(self, msg):
        # Completely ignore pit detection if in alignment mode
        if self.alignment_active:
            return
            
        if msg.data in ['Front_Left', 'Front_Right']:
            self.front_obstacle = True
            self.last_obstacle_direction = msg.data
            self.avoidance_stage = 'ANGULAR'
            self.get_logger().info(f"Pit detected: {msg.data}")
            self.waiting_for_next_stage = True
        elif msg.data in ['Front', 'Left', 'Right'] and self.waiting_for_next_stage:
            self.avoidance_stage = 'LINEAR'
            self.waiting_for_next_stage = False
            self.get_logger().info(f"Proceeding to linear stage: {msg.data}")

    def is_navigation_in_progress(self):
        if any([self.direction_command, self.distance_command, self.turn_command]):
            return True
        return False

    def avoid_obstacle(self):
        # Completely disable during alignment or ongoing navigation commands
        if self.alignment_active or self.is_navigation_in_progress():
            status_msg = String()
            status_msg.data = 'disable'
            self.obstacle_avoidance_status_publisher.publish(status_msg)
            return

        current_time = time.time()
        status_msg = String()

        # Angular stage when Front_Left or Front_Right is detected
        if self.avoidance_stage == 'ANGULAR':
            status_msg.data = 'enable'
            self.obstacle_avoidance_status_publisher.publish(status_msg)
            if self.front_obstacle:
                self.get_logger().info("Performing angular motion")
                self.cmd_vel_msg.linear.x = 0.0
                # Rotate based on the last detected obstacle direction
                self.cmd_vel_msg.angular.z = 0.9 if 'Front_Right' in str(self.last_obstacle_direction) else -0.9
                self.cmd_vel_publisher.publish(self.cmd_vel_msg)

        # Linear motion stage 
        elif self.avoidance_stage == 'LINEAR':
            status_msg.data = 'enable'
            self.obstacle_avoidance_status_publisher.publish(status_msg)
            self.get_logger().info("Starting linear motion")
            self.cmd_vel_msg.linear.x = 1.0
            self.cmd_vel_msg.angular.z = 0.0
            self.linear_motion_start_time = current_time
            self.avoidance_stage = 'forward_wait'
            self.cmd_vel_publisher.publish(self.cmd_vel_msg)

        elif self.avoidance_stage == 'forward_wait':
            status_msg.data = 'enable'
            self.obstacle_avoidance_status_publisher.publish(status_msg)
            if current_time - self.linear_motion_start_time >= 2.0:
                self.avoidance_stage = 'yaw_correct'

        # Yaw correction stage
        elif self.avoidance_stage == 'yaw_correct':
            status_msg.data = 'enable'
            self.obstacle_avoidance_status_publisher.publish(status_msg)
            self.get_logger().info("Yaw corrected, resetting state")
            self.cmd_vel_msg.linear.x = 1.0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(self.cmd_vel_msg)
            self.reset_state()
            
            status_msg.data = 'disable'
            self.obstacle_avoidance_status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoider = ObstacleAvoider()
    rclpy.spin(obstacle_avoider)
    obstacle_avoider.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
     main()