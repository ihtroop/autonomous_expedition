import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32
import sys
import time

class Controller(Node):
    def __init__(self):
        super().__init__("controller")

        self.linear_velocity = 2.0  
        self.angular_velocity = 0.0
        self.straightpathturnleft = 0.5
        self.straightpathturnright = -0.5
        self.turnleft = 1.0
        self.turnright = -1.0
        self.current_state = "default"  
        self.default_velocity = 1.0
        
        self.previous_state = None
        self.previous_linear_vel = None
        self.previous_angular_vel = None

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
        
        self.direction_timestamp = None
        self.stop_time = 2.0  

        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        
        # Set up subscribers
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

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cmd_vel = Twist()

    def is_any_operation_active(self):
        return (
            self.alignment_in_progress or
            self.is_turning or
            self.turn_complete_received or
            self.direction_received or
            self.distance_received or
            self.obstacle_detected
        )

    def should_publish_cmd_vel(self):
        velocity_changed = (self.linear_velocity != self.previous_linear_vel or 
                          self.angular_velocity != self.previous_angular_vel)
        return velocity_changed

    def publish_cmd_vel(self):
       
        self.previous_state = self.current_state
        self.previous_linear_vel = self.linear_velocity
        self.previous_angular_vel = self.angular_velocity
        
        
        self.cmd_vel.linear.x = self.linear_velocity
        self.cmd_vel.angular.z = self.angular_velocity
        self.cmd_vel_publisher.publish(self.cmd_vel)
        self.get_logger().info(f"Published cmd_vel - linear: {self.linear_velocity}, angular: {self.angular_velocity}")

    def obstacle_callback(self, msg):
        if self.alignment_in_progress or self.is_turning:
            return
            
        self.obstacle_direction = msg.data
        if self.obstacle_direction in ['Front_Left', 'Front_Right']:
            self.obstacle_detected = True
            self.linear_velocity = 0.0 
            self.angular_velocity = self.turnright if self.obstacle_direction == 'Front_Right' else self.turnleft
        else:
            self.obstacle_detected = False
            
            if self.current_state == "default":
                self.linear_velocity = self.default_velocity
                self.angular_velocity = 0.0
       
        self.publish_cmd_vel()

    def align_callback(self, msg):
        self.alignment_in_progress = True
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        
        self.is_turning = False
        self.direction_received = False
        self.straight_path_no = False
        self.direction_timestamp = None
        self.obstacle_detected = False

    def direction_callback(self, msg):
        if self.alignment_in_progress:
            return

        self.direction_received = True
        self.is_turning = True
        self.direction = msg.data
        self.direction_timestamp = time.time()

    def distance_callback(self, msg):
        if self.alignment_in_progress:
            return
        self.distance_received = True

    def turn_complete_callback(self, msg):
        if self.alignment_in_progress:
            return

        self.turn_complete_received = True
        self.is_turning = False
        self.direction_received = False
        self.direction_timestamp = None
        self.obstacle_detected = False

    def stop_callback(self, msg):
        self.stop_received = True
        self.direction_timestamp = None

    def straight_path_callback(self, msg):
        if not self.is_any_operation_active():
            self.straight_path_no = True
            self.deviation = msg.data

    def timer_callback(self):
        if self.stop_received:
            self.current_state = "STOPPED"
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0

        elif self.alignment_in_progress:
            self.current_state = "ALIGNING"
            self.alignment_in_progress = False

        elif self.is_turning:
            self.current_state = "TURNING"
            if self.direction_timestamp and (time.time() - self.direction_timestamp < self.stop_time):
                self.linear_velocity = 0.0
                self.angular_velocity = 0.0
            else:
                if self.direction == "Right":
                    self.linear_velocity = 0.0
                    self.angular_velocity = self.turnright
                elif self.direction == "Left":
                    self.linear_velocity = 0.0
                    self.angular_velocity = self.turnleft

        elif self.turn_complete_received:
            self.current_state = "MOVING"
            self.linear_velocity = 1.0
            self.angular_velocity = 0.0
            self.turn_complete_received = False

        elif self.straight_path_no and not self.is_any_operation_active():
            self.current_state = "MOVING"
            self.linear_velocity = 0.0
            self.angular_velocity = self.straightpathturnright if self.deviation < 0 else self.straightpathturnleft
            self.straight_path_no = False

        else:
            if not self.is_turning:
                self.current_state = "default"
                if not self.obstacle_detected:
                    self.linear_velocity = self.default_velocity
                    self.angular_velocity = 0.0
                    # self.linear_velocity = 1.0
                    # self.angular_velocity = 0.0

        if self.should_publish_cmd_vel():
            self.publish_cmd_vel()

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Keyboard Interrupt Ctrl+C")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()