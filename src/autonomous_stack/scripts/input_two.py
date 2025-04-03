# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32, String
# from geometry_msgs.msg import Twist
# import pyrealsense2 as rs
# import numpy as np
# import pygame
# import math

# #initializing the acceleration of gravity
# g = 9.80665

# #alpha vaue for complementary filter
# alpha2 = 0.01
# class IMUVisualizer(Node):
#     def __init__(self):
#         super().__init__('imu_visualizer')

#         # Publishers
#         self.turn_publisher = self.create_publisher(Int32, 'turn', 10)
#         # self.straight_path_publisher = self.create_publisher(Int32, 'straight_path', 10)

      
#         self.direction_subscriber = self.create_subscription(
#             String,
#             '/direction',
#             self.direction_callback,
#             10
#         )
#         self.align_subscriber = self.create_subscription(
#             Twist,
#             '/align_publisher',
#             self.align_callback,
#             10
#         )

#         self.pipeline = rs.pipeline()
#         config = rs.config()
#         config.enable_stream(rs.stream.accel)
#         config.enable_stream(rs.stream.gyro)
#         self.pipeline.start(config)

#         self.pitch = 0.0
#         self.prev_pitch = 0.0
#         self.roll = 0.0
#         self.dt = 0.1
#         self.gyro_drift_threshold = 0.01  

#         self.deviation_threshold = 10.0
#         self.turn_threshold = 90.0
#         self.straight_path_deviation = 0.0

#         self.alignment_in_progress = False
#         self.turn_detected = False

#         # Pygame GUI
#         pygame.init()
#         self.screen = pygame.display.set_mode((400, 300))
#         pygame.display.set_caption("IMU Data Visualization")
#         self.font = pygame.font.Font(None, 36)
#         self.clock = pygame.time.Clock()

#     def direction_callback(self, msg):
#         """Callback for direction messages that resets pitch"""
#         self.get_logger().info(f"Received direction: {msg.data}. Resetting pitch to zero.")
#         self.pitch = 0.0  

#     def align_callback(self, msg):
#         """Callback for alignment messages to manage alignment state"""
#         self.get_logger().info("Alignment topic received. Alignment in progress.")
#         self.alignment_in_progress = True

#     def accel_data_pitch_yaw(self, ax, ay, az):
#         #calculate theta(pitch)
#         ratio1 = ax/az if az > 0 else 0
#         accel_pitch = math.atan(ratio1)*(180./math.pi)
#         ratio2 = ax/g
#         accel_roll = math.asin(ratio2)*(180./math.pi)

#         return accel_pitch, accel_roll
    
#     def pqr_to_euler(self, p, q, r):
#         conversion_matrix = np.array([
#             [1, np.sin(self.roll)*np.tan(self.pitch), np.cos(self.roll)*np.tan(self.pitch)],
#             [0, np.cos(self.roll), -np.sin(self.roll)],
#             [0, np.sin(self.roll)/np.cos(self.pitch), np.cos(self.roll)/np.cos(self.pitch)]
#         ])
        
#         euler_rates = np.dot(conversion_matrix, [p,q,r])
#         return euler_rates

#     def get_motion_data(self):
#         """Get gyro and accelerometer data."""
#         try:
#             frames = self.pipeline.wait_for_frames()
#             gyro_frame = frames.first_or_default(rs.stream.gyro)
#             accel_frame = frames.first_or_default(rs.stream.accel)

#             if gyro_frame and accel_frame:
#                 gyro_data = gyro_frame.as_motion_frame().get_motion_data()
#                 accel_data = accel_frame.as_motion_frame().get_motion_data()
#                 return (gyro_data.x, gyro_data.y, gyro_data.z), (accel_data.x, accel_data.y, accel_data.z)
#             return None, None
#         except Exception as e:
#             self.get_logger().error(f"Error: {e}")
#             return None, None

#     def publish_turn_data(self):
#         msg = Int32()
#         msg.data = 1
#         self.turn_publisher.publish(msg)
#         self.turn_detected = True
#         self.get_logger().info("Turn Detected!")

#     # def check_straight_path(self, pitch_deg):
#     #     """Check and reset deviation, and publish deviation when it occurs."""
#     #     if not self.alignment_in_progress:
#     #         if abs(pitch_deg) < self.deviation_threshold:
#     #             self.straight_path_deviation = 0.0
#     #         else:
#     #             self.straight_path_deviation = pitch_deg  

#     #             msg = Int32()
#     #             msg.data = int(self.straight_path_deviation)
#     #             self.straight_path_publisher.publish(msg)
#     #             self.get_logger().info(f"Straight Path Deviation Published: {msg.data}")

#     def run(self):
#         running = True
#         try:
#             while rclpy.ok():
                
#                 rclpy.spin_once(self, timeout_sec=0)

#                 for event in pygame.event.get():
#                     if event.type == pygame.QUIT:
#                         running = False

#                 if not running:
#                     break

#                 gyro_data, accel_data = self.get_motion_data()
#                 if gyro_data and accel_data:
                    
#                     # msg = Float64()

#                     ax = accel_data[0]
#                     ay = accel_data[1]
#                     az = accel_data[2]

#                     gp = gyro_data[0]
#                     gq = gyro_data[1]
#                     gr = gyro_data[2]

                    
#                     accel_pitch, accel_roll = self.accel_data_pitch_yaw(ax, ay, az)


#                     alpha = 0.4
#                     # gyro_pitch_rate = gyro_data[1]  

#                     gyro_roll, gyro_pitch, gyro_yaw = self.pqr_to_euler(gp, gq, gr)

#                     gyro_roll = np.degrees(gyro_roll)
#                     gyro_pitch = np.degrees(gyro_pitch)
#                     gyro_yaw = np.degrees(gyro_yaw)

#                     if abs(gyro_pitch) < self.gyro_drift_threshold:
#                         self.pitch = accel_pitch

#                     self.pitch += gyro_pitch * self.dt

#                     # if abs(gyro_pitch_rate) > self.gyro_drift_threshold:
#                     #     self.pitch += gyro_pitch_rate * self.dt

#                     #complementarry filter
#                     pitch_value = alpha2*accel_pitch + (1-alpha2)*gyro_pitch
#                     roll_value = alpha2*accel_roll + (1-alpha2)*gyro_roll

#                     # pitch_deg = np.degrees(self.pitch)

#                     pitch_value = alpha*self.prev_pitch + (1-alpha)*self.pitch

#                     # Check for straight path deviation
#                     # self.check_straight_path(pitch_value)

#                     # Detect turn based on threshold
#                     if abs(pitch_value) >= self.turn_threshold:
#                         self.publish_turn_data()
#                         self.pitch = 0.0  
#                         self.prev_pitch = 0.0

#                     # Pygame display
#                     self.screen.fill((0, 0, 0))
#                     pitch_text = self.font.render(f"Pitch: {pitch_value:.2f}°", True, (255, 255, 255))
#                     deviation_text = self.font.render(f"Deviation: {self.straight_path_deviation:.2f}", True, (255, 255, 255))

#                     self.prev_pitch = pitch_value
#                     self.roll = roll_value

#                     # msg.data = float(np.float64(pitch_value))
#                     # self.pitch_publisher.publish(msg)

#                     self.screen.blit(pitch_text, (20, 50))
#                     self.screen.blit(deviation_text, (20, 100))
#                     pygame.display.flip()

#                 self.clock.tick(10)

#         except KeyboardInterrupt:
#             self.get_logger().info("Program interrupted.")
#         finally:
#             self.pipeline.stop()
#             pygame.quit()

# def main(args=None):
#     rclpy.init(args=args)
#     node = IMUVisualizer()
#     node.run()
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Int32
import pyrealsense2 as rs
import numpy as np
import math
from scipy.signal import butter, lfilter
import pygame

# Low-pass filter
def butter_lowpass_filter(data, cutoff, fs, order=4):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return lfilter(b, a, data)

class IMUFusionNode(Node):
    def __init__(self):
        super().__init__('imu_fusion_node')

        # # ROS Publishers
        # self.roll_pub = self.create_publisher(Float32, '/roll', 10)
        # self.pitch_pub = self.create_publisher(Float32, '/pitch', 10)
        self.yaw_pub = self.create_publisher(Float32, '/yaw', 10)
        self.turn_publisher = self.create_publisher(Int32, 'turn', 10)

        self.direction_subscriber = self.create_subscription(String, "/direction", self.direction_callback, 10)
        self.align_subscriber = self.create_subscription(String, "/align_publisher", self.align_callback, 10)
        self.obstacle_subscriber = self.create_subscription(String,'/obstacle_direction',self.obstacle_direction_callback,10)

        # RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.accel)
        config.enable_stream(rs.stream.gyro)
        self.pipeline.start(config)

        # Complementary filter parameters
        self.alpha = 0.025  # Trust factor for gyro over accel
        self.dt = 1 / 30.0  # Sampling time (RealSense IMU ~30Hz)

        # Calibration biases (replace with your calibration values)
        self.accel_bias = np.array([0.00023653, -0.00364435, 0.00017761])  # Replace with calibration offsets
        self.gyro_bias = np.array([0.00023653, -0.00364435, 0.00017761])   # Replace with calibration offsets

        # Low-pass filter parameters
        self.cutoff_freq = 5.0  # Hz
        self.sample_rate = 30.0  # Approx. RealSense IMU frequency
        self.gyro_filtered = np.zeros(3)
        self.accel_filtered = np.zeros(3)

        # Pygame GUI setup
        pygame.init()
        self.screen = pygame.display.set_mode((800, 400))
        pygame.display.set_caption("IMU Data Viewer")
        self.font = pygame.font.Font(None, 36)
        self.reset_button_rect = pygame.Rect(50, 300, 200, 50)

        # State variables
        self.yaw = 0.0
        self.obstacle_yaw = 0.0
        self.prev_yaw = 0.0
        self.prev_obstacle_yaw = 0.0

        # publisher constants
        self.deviation_threshold = 10.0
        self.turn_threshold = 90.0
        self.straight_path_deviation = 0.0

        # topic booleans
        self.alignment_in_progress = False
        self.turn_detected = False

        # New state tracking variables for obstacle avoidance
        self.front_left_seen = False
        self.front_right_seen = False
        self.last_direction = None

    def obstacle_direction_callback(self, msg):
        """Callback for obstacle direction messages with special reset logic"""
        if msg.data == "Front_Left":
            if not self.front_left_seen or self.last_direction == "Front_Right":
                self.get_logger().info("First Front_Left or after Front_Right. Resetting obstacle pitch to zero.")
                self.obstacle_yaw = 0.0
                self.front_left_seen = True
                self.front_right_seen = False
                self.last_direction = "Front_Left"
            else:
                self.get_logger().info("Subsequent Front_Left. Not resetting pitch.")
        
        elif msg.data == "Front_Right":
            if not self.front_right_seen or self.last_direction == "Front_Left":
                self.get_logger().info("First Front_Right or after Front_Left. Resetting obstacle pitch to zero.")
                self.obstacle_yaw = 0.0
                self.front_right_seen = True
                self.front_left_seen = False
                self.last_direction = "Front_Right"
            else:
                self.get_logger().info("Subsequent Front_Right. Not resetting pitch.")

    def direction_callback(self, msg):
        """Callback for direction messages that resets pitch"""
        self.get_logger().info(f"Received direction: {msg.data}. Resetting pitch to zero.")
        self.yaw = 0.0
    
    def align_callback(self, msg):
        """Callback for alignment messages to manage alignment state"""
        self.get_logger().info("Alignment topic received. Alignment in progress.")
        self.alignment_in_progress = True

    def scale_yaw(self, yaw):
        scale_factor = 67/90
        return yaw * scale_factor

    def process_imu_data(self, accel_raw, gyro_raw):
        # Apply calibration biases
        accel_calibrated = accel_raw - self.accel_bias
        gyro_calibrated = gyro_raw - self.gyro_bias

        # Low-pass filtering
        self.accel_filtered = butter_lowpass_filter(accel_calibrated, self.cutoff_freq, self.sample_rate)
        self.gyro_filtered = butter_lowpass_filter(gyro_calibrated, self.cutoff_freq, self.sample_rate)

        # Compute roll and pitch from accelerometer
        # roll_accel = math.atan2(self.accel_filtered[1], self.accel_filtered[2])
        # pitch_accel = math.atan2(-self.accel_filtered[0], 
                                #  math.sqrt(self.accel_filtered[1]**2 + self.accel_filtered[2]**2))

        # Complementary filter for roll and pitch
        # self.roll = self.alpha * (self.prev_roll + self.gyro_filtered[0] * self.dt) + (1 - self.alpha) * roll_accel
        # self.pitch = self.alpha * (self.prev_pitch + self.gyro_filtered[1] * self.dt) + (1 - self.alpha) * pitch_accel

        # Integrate yaw using gyro and normalize to [0, 360)
        self.yaw += self.gyro_filtered[2] * self.dt
        self.yaw %= (8.44006)
        # self.obstacle_yaw = self.yaw

        # Save previous values for next iteration
        # self.prev_roll = self.roll
        # self.prev_pitch = self.pitch
        self.prev_yaw = self.yaw
        # self.prev_obstacle_yaw = self.obstacle_yaw

        return self.yaw
    
    def reset_yaw(self):
        self.yaw = 0.0
        self.front_left_seen = False
        self.front_right_seen = False
        self.last_direction = None
        self.get_logger().info("All pitch values and states reset to zero.")

    def update_gui(self,yaw):
        self.screen.fill((0, 0, 0))  # Black background

        # Render text
        # roll_text = self.font.render(f"Roll: {math.degrees(roll):.2f}°", True, (255, 255, 255))
        # pitch_text = self.font.render(f"Pitch: {math.degrees(pitch):.2f}°", True, (255, 255, 255))
        yaw_text = self.font.render(f"Yaw: {math.degrees(yaw):.2f}°", True, (255, 255, 255))
        reset_text = self.font.render("Reset", True, (255, 255, 255))

        # Display text
        # self.screen.blit(roll_text, (50, 50))
        # self.screen.blit(pitch_text, (50, 100))
        self.screen.blit(yaw_text, (50, 150))

        # Draw reset button
        pygame.draw.rect(self.screen, (0, 128, 255), self.reset_button_rect)
        self.screen.blit(reset_text, (self.reset_button_rect.x + 50, self.reset_button_rect.y + 10))

        pygame.display.flip()

    def run(self):
        self.get_logger().info("Starting IMU fusion node...")
        try:
            while rclpy.ok():
                frames = self.pipeline.wait_for_frames()
                gyro_frame = frames.first_or_default(rs.stream.gyro)
                accel_frame = frames.first_or_default(rs.stream.accel)

                if gyro_frame and accel_frame:
                    gyro = np.array([gyro_frame.as_motion_frame().get_motion_data()[0],
                                     gyro_frame.as_motion_frame().get_motion_data()[1],
                                     gyro_frame.as_motion_frame().get_motion_data()[2]])
                    accel = np.array([accel_frame.as_motion_frame().get_motion_data()[0],
                                      accel_frame.as_motion_frame().get_motion_data()[1],
                                      accel_frame.as_motion_frame().get_motion_data()[2]])

                    yaw = self.process_imu_data(accel, gyro)

                    yaw = self.scale_yaw(self.yaw)

                    self.obstacle_yaw = yaw

                    if abs(yaw) >= self.turn_threshold:
                        self.turn_publisher.publish(Int32(1))
                        self.turn_detected = True
                        self.pitch = 0.0
                        self.get_logger().info("Turn Detected!")

                    if self.turn_detected:
                        self.alignment_in_progress = False
                        self.turn_detected = False

                    

                    # Publish values
                    # self.roll_pub.publish(Float32(data=math.degrees(roll)))
                    # self.pitch_pub.publish(Float32(data=math.degrees(pitch)))
                    self.yaw_pub.publish(Float32(data=math.degrees(yaw)))

                    # Update GUI
                    self.update_gui(yaw)

                # Handle GUI events
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        raise KeyboardInterrupt
                    if event.type == pygame.MOUSEBUTTONDOWN:
                        # Check if the reset button is clicked
                        if self.reset_button_rect.collidepoint(event.pos):
                            # Reset the IMU data
                            self.roll = 0.0
                            self.pitch = 0.0
                            self.yaw = 0.0
                            self.prev_roll = 0.0
                            self.prev_pitch = 0.0
                            self.prev_yaw = 0.0
                            self.get_logger().info("IMU data reset.")

        except KeyboardInterrupt:
            self.get_logger().info("Shutting down IMU fusion node.")
        finally:
            self.pipeline.stop()
            pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    node = IMUFusionNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()