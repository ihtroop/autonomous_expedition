import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Float32
from geometry_msgs.msg import Twist
import pyrealsense2 as rs
import numpy as np
import pygame

class IMUVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')

        # Publishers
        self.turn_publisher = self.create_publisher(Int32, 'turn', 10)
        self.straight_path_publisher = self.create_publisher(Int32, 'straight_path', 10)
        self.yaw_pubslisher = self.create_publisher(Float32, 'obstacle_yaw', 10)

        # Subscribers
        self.direction_subscriber = self.create_subscription(
            String,
            '/direction',
            self.direction_callback,
            10
        )
       
        self.align_subscriber = self.create_subscription(
            Twist,
            '/align_publisher',
            self.align_callback,
            10
        )

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.accel)
        config.enable_stream(rs.stream.gyro)
        self.pipeline.start(config)

        self.pitch = 0.0
        self.obstacle_pitch = 0.0  
        self.dt = 0.1
        self.gyro_drift_threshold = 0.1
        self.gyro_drift_threshold_obstacle = 0.075

        self.deviation_threshold =5.0
        self.turn_threshold = 80.0
        self.straight_path_deviation = 0.0

        self.alignment_in_progress = False
        self.turn_detected = False

        # New state tracking variables
        self.front_left_seen = False
        self.front_right_seen = False
        self.last_direction = None
        
        # Pygame GUI
        pygame.init()
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("IMU Data Visualization")
        self.font = pygame.font.Font(None, 36)
        self.clock = pygame.time.Clock()

        # Button properties
        self.button_rect = pygame.Rect(150, 200, 100, 50)
        self.button_color = (0, 255, 0)
        self.button_hover_color = (0, 200, 0)
        self.button_text = "Reset Pitch"

    def direction_callback(self, msg):
        """Callback for direction messages that resets pitch"""
        self.get_logger().info(f"Received direction: {msg.data}. Resetting pitch to zero.")
        

    def align_callback(self, msg):
        """Callback for alignment messages to manage alignment state"""
        self.get_logger().info("Alignment topic received. Alignment in progress.")
        self.alignment_in_progress = True

    def get_motion_data(self):
        """Get gyro and accelerometer data."""
        try:
            frames = self.pipeline.wait_for_frames()
            gyro_frame = frames.first_or_default(rs.stream.gyro)
            accel_frame = frames.first_or_default(rs.stream.accel)

            if gyro_frame and accel_frame:
                gyro_data = gyro_frame.as_motion_frame().get_motion_data()
                accel_data = accel_frame.as_motion_frame().get_motion_data()
                return (gyro_data.x, gyro_data.y, gyro_data.z), (accel_data.x, accel_data.y, accel_data.z)
            return None, None
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            return None, None

    def publish_turn_data(self):
        msg = Int32()
        msg.data = 1
        self.turn_publisher.publish(msg)
        self.turn_detected = True
        self.get_logger().info("Turn Detected!")

    def check_straight_path(self, pitch_deg):
         if not self.alignment_in_progress:
            if abs(pitch_deg) < self.deviation_threshold:
                self.straight_path_deviation = 0.0
            else:
                self.straight_path_deviation = pitch_deg

                msg = Int32()
                msg.data = int(self.straight_path_deviation)
                #self.straight_path_publisher.publish(msg)
                self.get_logger().info(f"Straight Path Deviation Published: {msg.data}")

    def reset_pitch(self):
        self.pitch = 0.0
        self.obstacle_pitch = 0.0
        self.front_left_seen = False
        self.front_right_seen = False
        self.last_direction = None
        self.get_logger().info("All pitch values and states reset to zero.")

    def run(self):
        running = True
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0)

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        if self.button_rect.collidepoint(event.pos):
                            self.reset_pitch()

                if not running:
                    break

                gyro_data, accel_data = self.get_motion_data()
                if gyro_data and accel_data:
                    alpha = 0.05
                    alpha_obstacle = 0.09
                    gyro_pitch_rate = gyro_data[1]

                    if abs(gyro_pitch_rate) > self.gyro_drift_threshold_obstacle:
                        # Update obstacle pitch
                        self.obstacle_pitch += gyro_pitch_rate * self.dt

                        # Synchronize pitch with obstacle pitch
                        self.pitch = self.obstacle_pitch

                    # Convert to degrees
                    prev_pitch = 0
                    obstacle_pitch_deg = np.degrees(self.obstacle_pitch)
                    pitch_value = alpha_obstacle* prev_pitch + (1 - alpha_obstacle) * obstacle_pitch_deg

                    msg = Float32()
                    msg.data = pitch_value
                    self.yaw_pubslisher.publish(msg)
                    self.get_logger().info(f"Yaw Published: {msg.data}")

                    self.check_straight_path(pitch_value)

                    if abs(abs(pitch_value) - self.turn_threshold) <= 3.0:
                        self.publish_turn_data()
                        self.pitch = 0.0
                        self.obstacle_pitch = 0.0  # Reset obstacle pitch after a turn is detected

                    if self.turn_detected:
                        self.alignment_in_progress = False
                        self.turn_detected = False

                    # Update GUI
                    # Update GUI
                    self.screen.fill((0, 0, 0))

                    # Render pitch and obstacle_pitch
                    pitch_text = self.font.render(f"Pitch: {pitch_value:.2f}°", True, (255, 255, 255))
                    obstacle_pitch_text = self.font.render(f"Obstacle Pitch: {obstacle_pitch_deg:.2f}°", True, (255, 255, 255))
                    deviation_text = self.font.render(f"Deviation: {self.straight_path_deviation:.2f}", True, (255, 255, 255))

                    # Display texts on screen
                    self.screen.blit(pitch_text, (20, 50))
                    self.screen.blit(obstacle_pitch_text, (20, 100))
                    self.screen.blit(deviation_text, (20, 150))

                    # Handle button hover and display
                    if self.button_rect.collidepoint(pygame.mouse.get_pos()):
                        pygame.draw.rect(self.screen, self.button_hover_color, self.button_rect)
                    else:
                        pygame.draw.rect(self.screen, self.button_color, self.button_rect)

                    button_text = self.font.render(self.button_text, True, (255, 255, 255))
                    self.screen.blit(button_text, (self.button_rect.x + 10, self.button_rect.y + 10))

                    pygame.display.flip()

                    prev_pitch = pitch_value

                self.clock.tick(10)

        except KeyboardInterrupt:
            self.get_logger().info("Program interrupted.")
        finally:
            self.pipeline.stop()
            pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    node = IMUVisualizer()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
