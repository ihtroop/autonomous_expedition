import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32
import sys
import time

class Controller(Node):
    def __init__(self):
        super().__init__("controller")

        # Previous state tracking for cmd_vel publishing
        self.prev_linear_velocity = 0.0
        self.prev_angular_velocity = 0.0
        self.prev_state = None

        # Velocity configurations
        self.linear_velocity = 2.0  
        self.angular_velocity = 0.0
        self.straightpathturnleft = 0.50
        self.straightpathturnright = -0.50
        self.turnleft = 0.70
        self.turnright = -0.70
        self.current_state = "default"  
        self.default_velocity = 0.75

        # State flags
        self.alignment_in_progress = False
        self.direction_received = False
        self.distance_received = False
        self.turn_complete_received = False
        self.stop_received = False
        self.straight_path_no = False
        self.deviation = 0.0
        self.direction = None
        self.is_turning = False
        self.obstacle_detected = False
        self.obstacle_direction = None
        self.pit_detected = False
        self.pit_direction = None
        self.obstacle_avoidance_active = False
        
        # Timing configurations
        self.direction_timestamp = None
        self.stop_time = 10.0  

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        
        # Subscribers
        self.direction_subscriber = self.create_subscription(
            String, "direction", self.direction_callback, 10
        )
        self.distance_subscriber = self.create_subscription(
            Int32, "distance", self.distance_callback, 10
        )
        self.turn_complete_subscriber = self.create_subscription(
            Int32, "turn", self.turn_complete_callback, 10
        )
        self.stop_subscriber = self.create_subscription(
            String, "stop_command", self.stop_callback, 10
        )
        self.align_subscriber = self.create_subscription(
            Twist, "/align_publisher", self.align_callback, 10
        )
        self.straight_path_subscriber = self.create_subscription(
            Int32, "straight_path", self.straight_path_callback, 10
        )
        self.obstacle_subscriber = self.create_subscription(
            String, "/obstacle_direction", self.obstacle_callback, 10
        )
        self.obstacle_avoidance_status_subscriber = self.create_subscription(
            String, "/obstacle_avoidance_status", self.obstacle_avoidance_status_callback, 10
        )
        self.pit_subscriber = self.create_subscription(
            String, "/pit_direction", self.pit_callback, 10
        )

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cmd_vel = Twist()

    
    def publish_if_changed(self):
        
        if (self.linear_velocity != self.prev_linear_velocity or 
            self.angular_velocity != self.prev_angular_velocity or
            self.current_state != self.prev_state):
            
            self.cmd_vel.linear.x = self.linear_velocity
            self.cmd_vel.angular.z = self.angular_velocity
            self.cmd_vel_publisher.publish(self.cmd_vel)

            self.prev_linear_velocity = self.linear_velocity
            self.prev_angular_velocity = self.angular_velocity
            self.prev_state = self.current_state

    def obstacle_avoidance_status_callback(self, msg):
        self.obstacle_avoidance_active = (msg.data == 'ACTIVE')
        self.get_logger().info(f"Obstacle avoidance status: {msg.data}")

    def is_any_operation_active(self):
        return (
            self.alignment_in_progress or
            self.is_turning or
            self.turn_complete_received or
            self.distance_received or
            self.direction_received or
            self.obstacle_detected or
            self.pit_detected or 
            self.obstacle_avoidance_active
        )

    def pit_callback(self, msg):
        if self.alignment_in_progress or self.is_turning:
            self.get_logger().info("Alignment or turning in progress")
            return
        self.pit_direction = msg.data
        if self.pit_direction in ['Front_Left', 'Front_Right']:
            self.pit_detected = True
            self.get_logger().info(f"Pit detected: {msg.data}")
        else:
            self.pit_detected = False
            self.pit_direction = None
            self.get_logger().info("No pit detected")

    def obstacle_callback(self, msg):
        if self.alignment_in_progress or self.is_turning:
            self.get_logger().info("Alignment in progress, ignoring obstacle")
            return
            
        self.obstacle_direction = msg.data
        if self.obstacle_direction in ['Front_Left', 'Front_Right']:
            self.obstacle_detected = True
            self.get_logger().info(f"Obstacle detected: {msg.data}")
        else:
            self.obstacle_detected = False
            self.obstacle_direction = None
            self.get_logger().info("No obstacle detected")

    def align_callback(self, msg):
        if msg.linear.x == 1.0:
            self.alignment_in_progress = False
            # Set velocity back to default values
            self.linear_velocity = self.default_velocity
            self.angular_velocity = 0.0
            self.get_logger().info("Alignment complete, returning to default velocity")
            
            # Publish the default velocity
            cmd_vel = Twist()
            cmd_vel.linear.x = self.default_velocity
            cmd_vel.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd_vel)
            return
            
        self.alignment_in_progress = True
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        
        # Reset other states
        self.is_turning = False
        self.direction_received = False
        self.straight_path_no = False
        self.direction_timestamp = None
        self.obstacle_detected = False 
        self.pit_detected = False
        
        self.get_logger().info("Alignment command received")
        
        # Publish alignment velocities
        cmd_vel = Twist()
        cmd_vel.linear.x = msg.linear.x
        cmd_vel.angular.z = msg.angular.z
        self.cmd_vel_publisher.publish(cmd_vel)

    def direction_callback(self, msg):
        if self.alignment_in_progress:
            self.get_logger().info("Alignment in progress, ignoring direction command")
            return

        self.direction_received = True
        self.is_turning = True
        self.direction = msg.data
        self.direction_timestamp = time.time()
        self.get_logger().info(f"Direction command received: {msg.data}")

    def distance_callback(self, msg):
        if self.alignment_in_progress:
            self.get_logger().info("Alignment in progress, ignoring distance")
            return

        self.distance_received = True
        self.get_logger().info("Distance update received")

    def turn_complete_callback(self, msg):
        if self.alignment_in_progress:
            self.get_logger().info("Alignment in progress, ignoring turn complete")
            return

        self.turn_complete_received = True
        self.is_turning = False
        self.direction_received = False
        self.direction_timestamp = None
        self.obstacle_detected = False 
        self.pit_detected = False
        self.get_logger().info("Turn complete signal received")

    def stop_callback(self, msg):
        self.stop_received = True
        self.direction_timestamp = None
        self.get_logger().info("Stop command received")

    def straight_path_callback(self, msg):
        if not self.is_any_operation_active():
            self.straight_path_no = True
            self.deviation = msg.data
            self.get_logger().info(f"Straight path correction received: {msg.data}")
        else:
            self.get_logger().info("Ignoring straight path correction due to active operation")

            
            

    def timer_callback(self):
        if not self.obstacle_avoidance_active:
            if self.stop_received:
                self.current_state = "STOPPED"
                self.linear_velocity = 0.0
                self.angular_velocity = 0.0
                self.get_logger().info("Robot STOPPED")

            elif self.alignment_in_progress:
                return

            elif self.is_turning:
                self.current_state = "TURNING"
                if self.direction_timestamp and (time.time() - self.direction_timestamp < self.stop_time):
                    self.linear_velocity = 0.0
                    self.angular_velocity = 0.0
                    self.get_logger().info("Stopping before turn")
                else:
                    if self.direction == "Right":
                        self.linear_velocity = 0.0
                        self.angular_velocity = self.turnright
                        self.get_logger().info("Turning Right")
                    elif self.direction == "Left":
                        self.linear_velocity = 0.0
                        self.angular_velocity = self.turnleft
                        self.get_logger().info("Turning Left")

            elif self.turn_complete_received:
                self.current_state = "MOVING"
                self.linear_velocity = 0.75
                self.angular_velocity = 0.0
                self.turn_complete_received = False
                self.get_logger().info("Turn complete, moving forward")

            elif self.straight_path_no and not self.is_any_operation_active():
                self.current_state = "MOVING"
                self.linear_velocity = 0.0
                self.angular_velocity = (
                    self.straightpathturnright if self.deviation < 0 
                    else self.straightpathturnleft
                )
                self.straight_path_no = False
                self.get_logger().info(f"Following straight path, deviation: {self.deviation}")

            else:
                if not self.is_turning:
                    self.current_state = "default"
                    if not self.obstacle_detected and not self.pit_detected:
                        self.linear_velocity = self.default_velocity
                        self.angular_velocity = 0.0
                        self.get_logger().info("Operating in default state")
                    else:
                        return
                    
            self.publish_if_changed()

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down due to keyboard interrupt")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()