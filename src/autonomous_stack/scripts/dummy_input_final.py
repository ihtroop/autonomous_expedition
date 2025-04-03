#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32
from nav_msgs.msg import Odometry
import tkinter as tk
from tkinter import ttk
import threading

class PublisherGUI(Node):
    def __init__(self):
        super().__init__('publisher_gui')
        
        # Create publishers
        self.align_pub = self.create_publisher(Twist, '/align_publisher', 10)
        self.direction_pub = self.create_publisher(String, '/direction', 10)
        self.distance_pub = self.create_publisher(Int32, '/distance', 10)
        self.stop_pub = self.create_publisher(String, '/stop_command', 10)
        self.turn_publisher = self.create_publisher(Int32, 'turn', 10)
        self.straight_path_publisher = self.create_publisher(Int32, 'straight_path', 10)
        self.obstacle_direction_pub = self.create_publisher(String, '/obstacle_direction', 10)
        self.pit_direction_pub = self.create_publisher(String, '/pit_direction', 10)
        
        # Create odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Initialize odom pose.x and pose.y
        self.pose_x = 0.0
        self.pose_y = 0.0
        
        # Initialize turn counter
        self.turn_count = 1
        
        # Create GUI window
        self.window = tk.Tk()
        self.window.title("ROS2 Publisher GUI")
        self.window.geometry("400x1000")
        
        # Create and pack widgets
        self.create_align_controls()
        self.create_direction_controls()
        self.create_distance_controls()
        self.create_stop_controls()
        self.create_turn_controls()
        self.create_straight_path_controls()
        self.create_obstacle_direction_controls()
        self.create_pit_direction_controls()
        self.create_odom_controls()

    def create_odom_controls(self):
        frame = ttk.LabelFrame(self.window, text="Odometry Controls", padding="10")
        frame.pack(fill="x", padx=10, pady=5)
        
        # Button to increment pose.x and pose.y
        ttk.Button(frame, text="Increment Pose.x & Pose.y", command=self.increment_pose).pack(pady=5)
        
        # Button to reset pose.y to zero
        ttk.Button(frame, text="Reset Pose.y to Zero", command=self.reset_pose_y).pack(pady=5)

    def increment_pose(self):
        self.pose_x += 1.0
        self.pose_y += 1.0
        
        # Create and publish Odometry message
        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = self.pose_x
        odom_msg.pose.pose.position.y = self.pose_y
        self.odom_pub.publish(odom_msg)
        print(f"Published Odometry: pose.x={self.pose_x}, pose.y={self.pose_y}")

    def reset_pose_y(self):
        self.pose_y = 0.0
        
        # Create and publish Odometry message with reset pose.y
        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = self.pose_x
        odom_msg.pose.pose.position.y = self.pose_y
        self.odom_pub.publish(odom_msg)
        print(f"Published Odometry with pose.y reset: pose.x={self.pose_x}, pose.y={self.pose_y}")

    def create_obstacle_direction_controls(self):
        frame = ttk.LabelFrame(self.window, text="Obstacle Direction", padding="10")
        frame.pack(fill="x", padx=10, pady=5)
        
        # Create buttons for each direction
        button_frame = ttk.Frame(frame)
        button_frame.pack()
        
        # Create first row of buttons
        row1_frame = ttk.Frame(button_frame)
        row1_frame.pack(pady=2)
        ttk.Button(row1_frame, text="Front", 
                  command=lambda: self.publish_obstacle_direction("Front")).pack(side="left", padx=5)
        
        # Create second row of buttons
        row2_frame = ttk.Frame(button_frame)
        row2_frame.pack(pady=2)
        ttk.Button(row2_frame, text="Front_Left", 
                  command=lambda: self.publish_obstacle_direction("Front_Left")).pack(side="left", padx=5)
        ttk.Button(row2_frame, text="Front_Right", 
                  command=lambda: self.publish_obstacle_direction("Front_Right")).pack(side="left", padx=5)
        
        # Create third row of buttons
        row3_frame = ttk.Frame(button_frame)
        row3_frame.pack(pady=2)
        ttk.Button(row3_frame, text="Left", 
                  command=lambda: self.publish_obstacle_direction("Left")).pack(side="left", padx=5)
        ttk.Button(row3_frame, text="Right", 
                  command=lambda: self.publish_obstacle_direction("Right")).pack(side="left", padx=5)

    def create_pit_direction_controls(self):
        frame = ttk.LabelFrame(self.window, text="Pit Direction", padding="10")
        frame.pack(fill="x", padx=10, pady=5)
        
        # Create buttons for each pit direction
        button_frame = ttk.Frame(frame)
        button_frame.pack()
        
        ttk.Button(button_frame, text="Front Right", 
                  command=lambda: self.publish_pit_direction("Front_Right")).pack(side="left", padx=5)
        ttk.Button(button_frame, text="Front Left", 
                  command=lambda: self.publish_pit_direction("Front_Left")).pack(side="left", padx=5)
        ttk.Button(button_frame, text="Front", 
                  command=lambda: self.publish_pit_direction("Front")).pack(side="left", padx=5)

    def create_align_controls(self):
        frame = ttk.LabelFrame(self.window, text="Align Publisher", padding="10")
        frame.pack(fill="x", padx=10, pady=5)
        
        # Linear velocity controls
        ttk.Label(frame, text="Linear Velocity (x, y, z):").pack()
        self.linear = []
        linear_frame = ttk.Frame(frame)
        linear_frame.pack()
        for i in range(3):
            entry = ttk.Entry(linear_frame, width=10)
            entry.insert(0, "0.0")
            entry.pack(side="left", padx=5)
            self.linear.append(entry)
            
        # Angular velocity controls
        ttk.Label(frame, text="Angular Velocity (x, y, z):").pack()
        self.angular = []
        angular_frame = ttk.Frame(frame)
        angular_frame.pack()
        for i in range(3):
            entry = ttk.Entry(angular_frame, width=10)
            entry.insert(0, "0.0")
            entry.pack(side="left", padx=5)
            self.angular.append(entry)

        ttk.Button(frame, text="Publish Twist", command=self.publish_twist).pack(pady=5)

    def create_direction_controls(self):
        frame = ttk.LabelFrame(self.window, text="Direction", padding="10")
        frame.pack(fill="x", padx=10, pady=5)
        
        button_frame = ttk.Frame(frame)
        button_frame.pack()
        ttk.Button(button_frame, text="Left", command=lambda: self.publish_direction("Left")).pack(side="left", padx=5)
        ttk.Button(button_frame, text="Right", command=lambda: self.publish_direction("Right")).pack(side="left", padx=5)

    def create_distance_controls(self):
        frame = ttk.LabelFrame(self.window, text="Distance", padding="10")
        frame.pack(fill="x", padx=10, pady=5)
        
        self.distance_entry = ttk.Entry(frame, width=10)
        self.distance_entry.insert(0, "0")
        self.distance_entry.pack(side="left", padx=5)
        ttk.Button(frame, text="Publish Distance", command=self.publish_distance).pack(side="left", padx=5)

    def create_stop_controls(self):
        frame = ttk.LabelFrame(self.window, text="Stop Command", padding="10")
        frame.pack(fill="x", padx=10, pady=5)
        
        self.stop_entry = ttk.Entry(frame, width=20)
        self.stop_entry.insert(0, "stop")
        self.stop_entry.pack(side="left", padx=5)
        ttk.Button(frame, text="Publish Stop", command=self.publish_stop).pack(side="left", padx=5)

    def create_turn_controls(self):
        frame = ttk.LabelFrame(self.window, text="Turn Counter", padding="10")
        frame.pack(fill="x", padx=10, pady=5)
        
        # Label to display current count
        self.turn_label = ttk.Label(frame, text=f"Current Count: {self.turn_count}")
        self.turn_label.pack(side="left", padx=5)
        
        ttk.Button(frame, text="Increment & Publish", command=self.publish_turn).pack(side="left", padx=5)

    def create_straight_path_controls(self):
        frame = ttk.LabelFrame(self.window, text="Straight Path Deviation", padding="10")
        frame.pack(fill="x", padx=10, pady=5)
        
        # Add entry for deviation value
        self.deviation_entry = ttk.Entry(frame, width=10)
        self.deviation_entry.insert(0, "0")
        self.deviation_entry.pack(side="left", padx=5)
        
        ttk.Button(frame, text="Publish Deviation", command=self.publish_straight_path).pack(side="left", padx=5)

    def publish_obstacle_direction(self, direction):
        msg = String()
        msg.data = direction
        self.obstacle_direction_pub.publish(msg)
        print(f"Published obstacle direction: {direction}")

    def publish_pit_direction(self, direction):
        msg = String()
        msg.data = direction
        self.pit_direction_pub.publish(msg)
        print(f"Published pit direction: {direction}")

    def publish_twist(self):
        msg = Twist()
        try:
            msg.linear.x = float(self.linear[0].get())
            msg.linear.y = float(self.linear[1].get())
            msg.linear.z = float(self.linear[2].get())
            msg.angular.x = float(self.angular[0].get())
            msg.angular.y = float(self.angular[1].get())
            msg.angular.z = float(self.angular[2].get())
            self.align_pub.publish(msg)
        except ValueError:
            print("Please enter valid numbers for Twist message")

    def publish_direction(self, direction):
        msg = String()
        msg.data = direction
        self.direction_pub.publish(msg)

    def publish_distance(self):
        try:
            msg = Int32()
            msg.data = int(self.distance_entry.get())
            self.distance_pub.publish(msg)
        except ValueError:
            print("Please enter a valid integer for distance")

    def publish_stop(self):
        msg = String()
        msg.data = self.stop_entry.get()
        self.stop_pub.publish(msg)

    def publish_turn(self):
        msg = Int32()
        msg.data = self.turn_count
        self.turn_publisher.publish(msg)
        self.turn_count += 1
        self.turn_label.config(text=f"Current Count: {self.turn_count}")

    def publish_straight_path(self):
        try:
            msg = Int32()
            msg.data = int(self.deviation_entry.get())
            self.straight_path_publisher.publish(msg)
        except ValueError:
            print("Please enter a valid integer for deviation")

    def run(self):
        self.window.mainloop()

def main():
    rclpy.init()
    gui = PublisherGUI()
    
    # Create a separate thread for ROS spinning
    ros_thread = threading.Thread(target=lambda: rclpy.spin(gui))
    ros_thread.daemon = True
    ros_thread.start()
    
    # Run the GUI
    gui.run()
    
    # Cleanup
    gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


