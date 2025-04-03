# for OpenCV
import cv2
import numpy as np
import pyrealsense2 as rs
import sys
import os
import supervision as sv
from ultralytics import YOLO
import math

# for realsense
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from miscellaneous import *

# For ROS
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int32
from time import time
import torch

dc = DepthCamera()

# Parameters for denoising and Canny edge detection
median_blur_ksize = 7
gaussian_blur_ksize = (9, 9)
canny_threshold1 = 50
canny_threshold2 = 150

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Device used: {device}")

class DirectionPublisher(Node):
    def __init__(self):
        super().__init__("direction_publisher")
        # initialising publishers and creating a timer to call send_cmd_vel function
        self.direction_publisher = self.create_publisher(String, "direction", 10)
        self.distance_threshold = self.create_publisher(Int32, "distance", 10)
        self.publisher_ = self.create_publisher(String, "/stop_command", 10)
        self.align_publisher = self.create_publisher(Twist, "/align_publisher", 10)
        self.create_timer(1, self.send_cmd_vel)
        self.get_logger().info("Direction Publisher node Chalu")

        # initialising some important variables
        self.prev_direction = None
        self.distance_published = False
        self.direction_published = False
        self.stop_published = False
        self.time_delay = 0.0
        self.last_distance_publish_time = None
        self.model = YOLO("/home/pradheep/integrated_ws_final/integrated_autonomous_stack/src/autonomous_stack/scripts/best_new_new.pt")  # this is the path to the weight file
        self.model.to(device)
        self.bounding_box_annotator = sv.BoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()
        self.cone_class_id = 0
        self.confidence_threshold = 0.8
        # self.FOCAL_LENGTH = 75
        # self.REAL_ARROW_WIDTH = 300
        self.area_threshold=0.4


        # initilising messages to send in topic
        self.vel_msg = Twist()
        self.align_vel_msg = Twist()

    def arrow_distance_estimation(self, valid, d_frame, cx, cy):
        # if width_in_frame != 0:
        #     print(width_in_frame)
        #     distance = (real_width * focal_length) / width_in_frame
        # else:
        #     distance = 0
        # return distance
        # valid, depth_image, _ = dc.get_frame()
        # if not valid:
        #     print("NOt valid")
        #     return None
        # if cx<0 or cx>=d_frame.shape[1] or cy<0 or cy>=d_frame.shape[0]:
        #     print("Coordinates are out of bounds")
        prev_dist = 0.0

        #     return None
        depth_value = d_frame[cy, cx]

        # print(f"Depth value at ({x}, {y}): {depth_value}")

        # if depth_value == 0:
        #     print("No valid measurement at this pixel.")
        #     return None

        distance = depth_value / 10
        if distance == 0.0:
            distance = prev_dist
        else:
            prev_dist = distance

        return distance