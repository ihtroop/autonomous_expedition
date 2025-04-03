#!/usr/bin/env python3
# for OpenCV
import cv2
import numpy as np
#import pyrealsense2 as rs
import sys
import os
import supervision as sv
from ultralytics import YOLO
import math
from sensor_msgs.msg import Image
import cv_bridge
from cv_bridge import CvBridge
import pickle
from pathos.multiprocessing import Pool
from concurrent.futures import ThreadPoolExecutor
from heapq import nlargest

# for realsense
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
#from miscellaneous import *

# For ROS
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int32
from time import time
import torch
from message_filters import Subscriber, ApproximateTimeSynchronizer

cv2.setUseOptimized(True)

# Constants initialisations for OpenCVeo
#dc = DepthCamera()
# dc = cv2.VideoCapture(0)

# Parameters for denoising and Canny edge detection
median_blur_ksize = 7  # Reduced kernel size for faster processing
gaussian_blur_ksize = (9, 9)  # Reduced kernel size for faster processing
canny_threshold1 = 50  # Adjusted threshold for faster processing
canny_threshold2 = 150  # Adjusted threshold for faster processing

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Device used: {device}")

# for better arrow tracking
class EuclideanDistTracker:
    def __init__(self):
        self.center_points = {}
        self.id_count = 0

    # this function is to update the arrows
    def update(self, objects_rect):
        objects_bbs_ids = []
        for rect in objects_rect:
            x, y, w, h, contour, direction = rect
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2
            centroid = (cx, cy)

            same_object_detected = False
            for id, pt in self.center_points.items():
                dist = math.hypot(cx - pt[0], cy - pt[1])

                if dist < 10:
                    self.center_points[id] = (cx, cy)
                    objects_bbs_ids.append(
                        [x, y, w, h, id, contour, centroid, direction]
                    )
                    same_object_detected = True
                    break

            if not same_object_detected:
                self.center_points[self.id_count] = (cx, cy)
                objects_bbs_ids.append(
                    [x, y, w, h, self.id_count, contour, centroid, direction]
                )
                self.id_count += 1

        new_center_points = {}
        for obj_bb_id in objects_bbs_ids:
            object_id = obj_bb_id[4]
            center = self.center_points[object_id]
            new_center_points[object_id] = center
        
        self.center_points = new_center_points.copy()
        return objects_bbs_ids

    """ def non_max_suppression(self, boxes, overlapThresh=0.3):
        if len(boxes) == 0:
            return []

        boxes = np.array(boxes)
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 0] + boxes[:, 2]
        y2 = boxes[:, 1] + boxes[:, 3]
        depth = boxes[:, 4]
        centroid = boxes[:, 5]
        ids = boxes[:, 6]

        area = (x2 - x1 + 1) * (y2 - y1 + 1)
        idxs = np.argsort(y2)

        pick = []
        while len(idxs) > 0:
            last = len(idxs) - 1
            i = idxs[last]
            pick.append(boxes[i])

            xx1 = np.maximum(x1[i], x1[idxs[:-1]])
            yy1 = np.maximum(y1[i], y1[idxs[:-1]])
            xx2 = np.minimum(x2[i], x2[idxs[:-1]])
            yy2 = np.minimum(y2[i], y2[idxs[:-1]])

            w = np.maximum(0, xx2 - xx1 + 1)
            h = np.maximum(0, yy2 - yy1 + 1)
            overlap = (w * h) / area[idxs[:-1]]

            idxs = np.delete(idxs, np.concatenate(([last], np.where(overlap > overlapThresh)[0])))

        return pick """

class DirectionPublisher(Node):
    def __init__(self):
        super().__init__("direction_publisher")
        self.height=1920
        self.width=1080
        # initialising publishers and creating a timer to call send_cmd_vel function
        self.direction_publisher = self.create_publisher(String, "direction", 10)
        self.distance_threshold = self.create_publisher(Int32, "distance", 10)
        self.publisher_ = self.create_publisher(String, "/stop_command", 10)
        self.align_publisher = self.create_publisher(Twist, "/align_publisher", 10)
        #self.create_timer(1, self.send_cmd_vel)
        self.get_logger().info("Direction Publisher node Chalu")
        # getting the frames from the subscribing to image raw cv2_bridge
        #self.color_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        #self.depth_sub = Subscriber(self, Image, '/camera/camera/depth/image_rect_raw')
        #self.sync = ApproximateTimeSynchronizer(
        #    [self.color_sub, self.depth_sub], queue_size=10, slop=0.1, allow_headerless=True
        #)
        #self.sync.registerCallback(self.synced_callback)
        self.bridge = CvBridge()
        self.ret=None
        self.frame=None
        # initialising some important variables
        self.prev_direction = None
        self.distance_published = False
        self.direction_published = False
        self.stop_published = False
        self.time_delay = 0.0
        self.last_distance_publish_time = None
        self.model = YOLO("/home/pradheep/integrated_autonomous_stack/src/autonomous_stack/scripts/best.pt")
        self.model.to(device)  # this is the path to the weight file
        self.bounding_box_annotator = sv.BoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()
        self.cone_class_id = 0
        self.confidence_threshold = 0.875
        #self.FOCAL_LENGTH = 75
        self.scaling_factor = 1000 / 1280
        self.REAL_ARROW_WIDTH = 30
        self.REAL_ARROW_HEIGHT =20
        self.border_size = 20
        self.video_port="/dev/video8"
        self.stop_dist_ll = 40
        self.stop_dist_ul = 70
        #self.arrow_detected = False  # Add this to the initialization of your class
        #self.d_frame=None
        # initilising messages to send in topic
        self.vel_msg = Twist()
        self.align_vel_msg = Twist()
        self.latest_color_msg = None
        self.latest_depth_msg = None
        # gst_pipeline = (
        #     "v4l2src device=/dev/video2 ! "
        #     "video/x-raw,width=1920,height=1080 ! "
        #     "videoconvert ! "
        #     "appsink"
        # )

        self.capture= cv2.VideoCapture(self.video_port)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.height)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.width)
        self.capture.set(cv2.CAP_PROP_FPS, 30)  # Higher frame rate
        # self.capture.set(cv2.CAP_PROP_EXPOSURE, -3)
        # self.capture.set(cv2.CAP_PROP_CONVERT_RGB, 1),  # Ensure RGB conversion
        # self.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)

        # self.gpu = cv2.ocl.haveOpenCL()
        self.capture_frame = 0
        # if self.gpu:
        #     print("Using opencl")
        # else:
        #     print("cannot use opencl")
        # if not self.capture.isOpened():
        #     raise Exception("Could not open video capture device")
        self.timer = self.create_timer(0.1, self.process_image)
        self.area_threshold=0.4

        # Timer for frame processing
        #self.timer = self.create_timer(0.1, self.process_frame)
    
    # def color_callback(self, color_msg): 
    #     new_frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
    #     self.frame = cv2.resize(new_frame, (1280, 720))
    #     self.latest_color_msg = color_msg
    #     self.process_image()
    # def depth_callback(self, depth_msg):
    #     new_d_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
    #     self.d_frame = cv2.resize(new_d_frame, (1280, 720))
    #     self.latest_depth_msg = depth_msg  
    #     self.process_image()
    """ def synced_callback(self, color_msg, depth_msg):
        self.frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        self.d_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')

        self.frame = cv2.resize(self.frame, (1280, 720))
        self.d_frame = cv2.resize(self.d_frame, (1280, 720))

        self.process_image() """
        
    def process_image(self):
         self.ret, self.frame = self.capture.read()
         if self.ret:
            # while self.capture.grab()
            #     pass
            # frame = cv2.UMat(self.frame)
            # self.capture_frame +=1
            # if self.capture_frame % 3 == 0:
                self.send_cmd_vel(self.ret, self.frame)
            # self.send_cmd_vel(self.ret,self.frame)
         else:
            print("Error: Could not read frame from webcam.")

    # def image_callback(self, img_msg):
    #     if img_msg.encoding == 'bgr8':  
    #         self.frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
    #         self.latest_color_msg = img_msg  
    #     elif img_msg.encoding == '16UC1':  
    #         self.d_frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='16UC1')
    #         self.latest_depth_msg = img_msg  

    #     if self.frame is not None and self.d_frame is not None:
    #         self.send_cmd_vel(self.frame, self.d_frame)
    def arrow_distance_estimation(self, w):
        # Estimate the Z-axis (depth from the camera)
        object_width_in_pixels = w  # Use width for depth estimation
        focal_length =(1.48302782e+04 + 3.49665637e+04)/2 # Focal length in pixels
        distance = (((self.REAL_ARROW_WIDTH * focal_length) / object_width_in_pixels  )*self.scaling_factor/25)# Depth in cm
        #print(depth)
        if distance <50:
            distance=distance*1.25
        elif distance<100 and distance>50:
            distance*=1.35
        else:
            pass
        return distance 
        # return distance 
        # depth_tensor = torch.from_numpy(d_frame).to(device).float()
        # cx = torch.clamp(torch.tensor(cx, device=device), 0, depth_tensor.shape[1] - 1).int()
        # cy = torch.clamp(torch.tensor(cy, device=device), 0, depth_tensor.shape[0] - 1).int()
        # depth_value = depth_tensor[cy, cx]
        # distance = depth_value / 10.0
        # if distance == 0.0:
        #     distance = prev_dist
        # return distance
    

    def control_turtlebot(self, contour, relative_position, distance):
        # Clear velocity message before setting new values
        self.align_vel_msg.linear.x = 0.0
        self.align_vel_msg.angular.z = 0.0
        cv2.line(self.frame, (1280//3, 0), (1280//3, 720), (255,0,0), 2)
        cv2.line(self.frame, (2*(1280//3), 0), (2*(1280//3), 720), (255,0,0), 2)
        #self.arrow_detected=False
        # Set angular velocity based on relative position (left/right)
        if self.stop_dist_ll < distance < self.stop_dist_ul:
            if relative_position[0] < -240:  # Move left
                self.align_vel_msg.angular.z = 1.0
                self.get_logger().info(f"{self.align_vel_msg}")
                self.get_logger().info(f"relative postition: {relative_position}")
                self.align_publisher.publish(self.align_vel_msg)
                #self.arrow_detected=False
            elif relative_position[0] > 240:  # Move right
                self.align_vel_msg.angular.z = -1.0
                self.get_logger().info(f"{self.align_vel_msg}")
                self.get_logger().info(f"relative postition: {relative_position}")
                self.align_publisher.publish(self.align_vel_msg)
                #self.arrow_detected=False
            else:
                self.align_vel_msg.angular.z = 0.0
                self.align_vel_msg.linear.x = 1.0
                self.get_logger().info(f"{self.align_vel_msg}")
                self.align_publisher.publish(self.align_vel_msg)
        else:
           pass
            # Publish the velocity command
    def calculate_brightness(self,roi):
        """
        Calculates the average brightness of a region of interest (ROI).
        
        Args:
            roi (numpy array): ROI in BGR format.

        Returns:
            float: Average brightness value (0-255).
        """
        # Convert the ROI to grayscale
        roi_tensor = torch.from_numpy(roi).float().mean(dim=-1)
        avg_brightness = roi_tensor.mean().item()
        return avg_brightness  

    def calculate_global_maxima_ratio_split(self,image, brightness_threshold):
        """
        Calculates variance and mean ratio of histogram peaks based on brightness.
        
        Args:
            image (numpy array): ROI in BGR format.
            brightness_threshold (float): Threshold for brightness.

        Returns:
            tuple: Variance and mean ratio based on the histogram split logic.
        """
        # ratios = {}
        # hist_split = (100, 255) if brightness_threshold < 130 else (150, 255)

        # for i, channel_name in enumerate(['Blue', 'Green', 'Red']):
        #     channel = image[:, :, i]
        #     hist = cv2.calcHist([channel], [0], None, [256], [0, 256]).flatten()

        #     hist_low = hist[:hist_split[0] + 1]
        #     hist_high = hist[hist_split[0] + 1:hist_split[1] + 1]

        #     global_max_low = max(hist_low)
        #     global_max_high = max(hist_high)

        #     ratio = global_max_low / global_max_high if global_max_high != 0 else 0
        #     ratios[channel_name] = ratio

        # ratio_values = list(ratios.values())
        # mean_ratio = np.mean(ratio_values)
        # variance = np.var(ratio_values)

        # return variance * 100, mean_ratio   
        image_tensor = torch.from_numpy(image).permute(2, 0, 1).float()
        hist_split = (100, 255) if brightness_threshold < 130 else (150, 255)

        def safe_channel_histogram(channel):
            hist = torch.histc(channel, bins = 256, min=0, max=255)
            hist_low = hist[:hist_split[0] + 1]
            hist_high = hist[hist_split[0] +1: hist_split[1] +1]
            global_max_low = torch.max(hist_low).item()
            global_max_high = torch.max(hist_high).item()
            return global_max_low / global_max_high if global_max_high != 0 else 0
        
        ratios = {
            'Blue': safe_channel_histogram(image_tensor[0]),
            'Green': safe_channel_histogram(image_tensor[1]),
            'Red': safe_channel_histogram(image_tensor[2])
        }
        
        ratio_values = list(ratios.values())
        mean_ratio = np.mean(ratio_values)
        variance = np.var(ratio_values)
        
        return variance * 100, mean_ratio
    
    def process_contour(self, contour, frame_with_border):
        if cv2.contourArea(contour) < 1000:  # Adjust threshold if needed
                return None

        # Approximate the contour to a polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        if len(approx) == 7:
            angles = self.calculate_angles(approx)
            angles.sort()
            tip_angle1, tip_angle2, tip_angle3 = angles[0], angles[1], angles[2]
            base_angle1, base_angle2 = angles[3], angles[4]
            head_angle1,head_angle2=angles[5],angles[6]
            print("tri",tip_angle1+tip_angle2+tip_angle3)
            print("rect",base_angle1+base_angle2+head_angle1+head_angle2)
            if (
                30 <= tip_angle1 <= 90 and
                70 <= base_angle1 <= 140 and
                70 <= base_angle2 <= 140 and
                30 <= tip_angle2 <= 90 and
                30 <= tip_angle3 <= 90 and
                70 <= head_angle1 <= 140 and
                70 <= head_angle2 <= 140  and
                150<=(tip_angle1+tip_angle2+tip_angle3)<=200 and
                330<=(base_angle1+base_angle2+head_angle1+head_angle2)<=400
            ):
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    # Compute the centroid of the contour
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centroid = (cx, cy)

                    # Compute the bounding box around the contour
                    x, y, w, h = cv2.boundingRect(contour)
                    roi = frame_with_border[y:y + h, x:x + w]

                    # Split the bounding box into four rectangles centered at the centroid
                    horizontal_split_line = y + (cy - y)
                    vertical_split_line = x + (cx - x)

                    farthest_point = max(
                        approx, key=lambda p: np.linalg.norm(p[0] - centroid)
                    )
                    tip = tuple(farthest_point[0])

                    # Determine which rectangle the tip lies in
                    if tip[1] < horizontal_split_line:  # Top half
                        if tip[0] < vertical_split_line:  # Top-left quadrant
                            direction_finalised = "Right"
                        else:  # Top-right quadrant
                            direction_finalised = "Left"
                    else:  # Bottom half
                        if tip[0] < vertical_split_line:  # Bottom-left quadrant
                            direction_finalised = "Right"
                        else:  # Bottom-right quadrant
                            direction_finalised = "Left"

                    # Further validation based on brightness and variance
                    brightness = self.calculate_brightness(roi)
                    variance, mean_ratio = self.calculate_global_maxima_ratio_split(roi, brightness)

                    #if variance != 0 and mean_ratio != 0 and variance < 3 and mean_ratio < 1.5:
                    return (x, y, w, h, contour, direction_finalised)
        return None
    
    def calculate_all_contours(self, contours, frame_with_border):
        with ThreadPoolExecutor(max_workers=6) as executor:
            results = list(executor.map(
                lambda contour: self.process_contour(contour, frame_with_border), 
                contours
            ))
        detected_arrows = [result for result in results if result is not None]
        return detected_arrows


    def calculate_angles(self, approx):
        def angle(pt1, pt2, pt3):
    # Convert to float tensors explicitly
            v1 = torch.tensor(pt1, dtype=torch.float32, device=device) - torch.tensor(pt2, dtype=torch.float32, device=device)
            v2 = torch.tensor(pt3, dtype=torch.float32, device=device) - torch.tensor(pt2, dtype=torch.float32, device=device)
            
            # Ensure floating-point calculations
            cosine_angle = torch.dot(v1, v2) / (torch.norm(v1) * torch.norm(v2))
            cosine_angle = torch.clamp(cosine_angle, -1.0, 1.0)
            return (torch.acos(cosine_angle) * 180 / torch.pi).item()

        angles = []
        for i in range(len(approx)):
            pt1 = approx[i - 1][0]
            pt2 = approx[i][0]
            pt3 = approx[(i + 1) % len(approx)][0]
            angles.append(angle(pt1, pt2, pt3))
        return angles

    def send_cmd_vel(self, ret,frame):
        # camMatrix = [[1.48302782e+04, 0.00000000e+00, 2.81691110e+02],
        #      [0.00000000e+00, 3.49665637e+04, 2.63776143e+02],
        #      [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
        # distCoeff = [[-4.58160346e+00, 1.01739350e+03, 5.14277252e-02, 
        #       -8.45131600e-01, -2.52585378e+04]]
        tracker = EuclideanDistTracker()
        # msg1 = String()
        # msg2 = Int32()
        # tracker = EuclideanDistTracker()
        # while True:
        # for img_msg to cv2
        """ if frame is None or d_frame is None:
            print("Error: One of the frames is None.")
            return """
        # cv2.imshow("ROS2 Camera Frame", frame)
        #cv2.waitKey(1)

    # Rest of the processing logic from send_cmd_vel
        msg1 = String()
        msg2 = Int32()
        tracker = EuclideanDistTracker()
    
            # for RealSense
            #ret, d_frame, frame = dc.get_frame()
            # for img_msg to cv2
            # ret, frame = dc.read()
        if not ret:
            print("Error: Could not read frame from webcam.")
            return
        frame_height, frame_width, _ = frame.shape
        frame_center_x = frame_width // 2
        frame_center_y = frame_height // 2
            #if not ret:
        """ if frame is None and d_frame is None:
                print("Error: Could not read frame from webcam.")
                return """
        #frame_with_border = cv2.UMat(frame)
        cv2.ocl.setUseOpenCL(True)

        frame_with_border = frame
        # frame_with_border_umat = cv2.UMat(frame_with_border)
        # if self.gpu:
        #     frame_with_border_umat = cv2.UMat(frame_with_border)
        #     frame_denoised = cv2.medianBlur(frame_with_border_umat, 3)
        #     frame_denoised = cv2.GaussianBlur(frame_denoised, (5, 5), 0)
        
        #     frame_denoised = frame_denoised.get()
        # else:
        #     frame_denoised = cv2.medianBlur(frame_with_border, 3)
        #     frame_denoised = cv2.GaussianBlur(frame_denoised, (5, 5), 0)

        # frame_with_border_umat = cv2.UMat(frame_with_border)
            # Apply median blur (stronger than Gaussian for denoising)
        frame_denoised = cv2.medianBlur(frame_with_border, median_blur_ksize)

        #     # Apply Gaussian blur
        frame_denoised = cv2.GaussianBlur(frame_denoised, gaussian_blur_ksize, 0)

            # Convert to grayscale
        gray = cv2.cvtColor(frame_denoised, cv2.COLOR_BGR2GRAY)

            # Apply adaptive thresholding
        binary = cv2.adaptiveThreshold(
                gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2
            )

            # Detect edges using Canny
        edges = cv2.Canny(binary, canny_threshold1, canny_threshold2)

            # Find contours
        contours, _ = cv2.findContours(
                edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

         # Detect arrows and calculate tip
        detected_arrows = self.calculate_all_contours(contours, frame_with_border)
        # for contour in contours:
            # Filter small contours
            # if cv2.contourArea(contour) < 1500:  # Adjust threshold if needed
            #     continue

            # # Approximate the contour to a polygon
            # epsilon = 0.02 * cv2.arcLength(contour, True)
            # approx = cv2.approxPolyDP(contour, epsilon, True)

            # if len(approx) == 7:
            #     angles = self.calculate_angles(approx)
            #     angles.sort()
            #     tip_angle1, tip_angle2, tip_angle3 = angles[0], angles[1], angles[2]
            #     base_angle1, base_angle2 = angles[3], angles[4]
            #     head_angle1,head_angle2=angles[5],angles[6]
            #     if (
            #         30 <= tip_angle1 <= 90 and
            #         70 <= base_angle1 <= 140 and
            #         70 <= base_angle2 <= 140 and
            #         30 <= tip_angle2 <= 90 and
            #         30 <= tip_angle3 <= 90 and
            #         70 <= head_angle1 <= 140 and
            #         70 <= head_angle2 <= 140 
            #     ):
            #         M = cv2.moments(contour)
            #         if M["m00"] != 0:
            #             # Compute the centroid of the contour
            #             cx = int(M["m10"] / M["m00"])
            #             cy = int(M["m01"] / M["m00"])
            #             centroid = (cx, cy)

            #             # Compute the bounding box around the contour
            #             x, y, w, h = cv2.boundingRect(contour)
            #             roi = frame_with_border[y:y + h, x:x + w]

            #             # Split the bounding box into four rectangles centered at the centroid
            #             horizontal_split_line = y + (cy - y)
            #             vertical_split_line = x + (cx - x)

            #             farthest_point = max(
            #                 approx, key=lambda p: np.linalg.norm(p[0] - centroid)
            #             )
            #             tip = tuple(farthest_point[0])

            #             # Determine which rectangle the tip lies in
            #             if tip[1] < horizontal_split_line:  # Top half
            #                 if tip[0] < vertical_split_line:  # Top-left quadrant
            #                     direction_finalised = "Right"
            #                 else:  # Top-right quadrant
            #                     direction_finalised = "Right"
            #             else:  # Bottom half
            #                 if tip[0] < vertical_split_line:  # Bottom-left quadrant
            #                     direction_finalised = "Left"
            #                 else:  # Bottom-right quadrant
            #                     direction_finalised = "Left"

            #             # Further validation based on brightness and variance
            #             brightness = self.calculate_brightness(roi)
            #             variance, mean_ratio = self.calculate_global_maxima_ratio_split(roi, brightness)

            #             #if variance != 0 and mean_ratio != 0 and variance < 3 and mean_ratio < 1.5:
            #             detected_arrows.append((x, y, w, h, contour, direction_finalised))
                # global boxes_ids
        # Initialize persistent storage for the closest arrow based on area
        if not hasattr(self, 'persistent_closest_arrow'):
            self.persistent_closest_arrow = None
            self.missed_frames = 0

        # Filter to keep only the arrow with the largest bounding box area
        # closest_arrow = None
        # largest_area = 0
        largest_arrows = nlargest(1, detected_arrows, key=lambda arrow: arrow[2] * arrow[3])
        closest_arrow = largest_arrows[0] if largest_arrows else None

        # Handle persistence logic
        if closest_arrow:
            self.persistent_closest_arrow = closest_arrow  # Update the persistent arrow
            self.missed_frames = 0  # Reset missed frames counter
        else:
            self.missed_frames += 1
            if self.missed_frames <= 10:  # Allow up to 3 missed frames before switching
                closest_arrow = self.persistent_closest_arrow
            else:
                self.persistent_closest_arrow = None  # Clear persistent data after threshold

        # Pass only the closest arrow (persistent or new) to the tracker
        if closest_arrow:
            boxes_ids = tracker.update([closest_arrow])
        else:
            boxes_ids = tracker.update([])  # No detection, clear the tracker

        #boxes_ids = tracker.update(detected_arrows)
        distance = 0
        for boxes in boxes_ids:
                x, y, w, h, id = boxes[:5]
                # Draw the arrow's contour
                # Calculate box area
                box_area = w * h
                # Calculate aspect ratio
                aspect_ratio = box_area / (1920*1080)
                #print(aspect_ratio)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                mid_x = x + w // 2
                mid_y = y + h // 2
                cv2.circle(frame, (mid_x, mid_y), 10, (0, 0, 255), -1)
                direction = str(boxes[-1])
                cv2.putText(
                    frame,
                    direction,
                    (mid_x, mid_y + 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (255, 0, 0),
                    2,
                )
                contour = boxes[5]
                centroid = boxes[6]
                relative_position = (
                    centroid[0] - frame_center_x,
                    centroid[1] - frame_height // 2,
                )
                distance = self.arrow_distance_estimation(w)
                self.control_turtlebot(contour, relative_position, distance)
                # Display the arrow's data
                #print(distance)
                print(aspect_ratio,distance)
                cv2.putText(
                    frame,
                    f"{distance:.3f}m",
                    (mid_x, mid_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 255),
                    2,
                )
                # for arrow tip
                # cv2.circle(frame, tip, 5, (0, 0, 255), -1)
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)


        if not self.distance_published and 60 <= distance <= 100:
                msg2.data = int(distance)
                self.distance_threshold.publish(msg=msg2)
                self.get_logger().info(f" Distance: {distance}")
                self.distance_published = True
                self.last_distance_publish_time = time()

        if (
                self.distance_published
                and not self.direction_published
                and self.last_distance_publish_time
                and time() - self.last_distance_publish_time >= self.time_delay
            ):
                msg1.data = direction
                self.direction_publisher.publish(msg=msg1)
                self.get_logger().info(f"Published direction: {direction}")
                self.direction_published = True

        if distance < 50 or distance > 120:
                self.distance_published = False
                self.direction_published = False
                self.last_distance_publish_time = None
 
        #self.check_Arrow_Detection(distance,direction)
            # else:
            #     buffer.clear()
            # Run YOLO model on the color image
            # results = self.model(frame)[0]
        results = self.model.predict(source=frame, device=device, verbose=False)[0]
        detections = sv.Detections.from_ultralytics(results)

        # Filter detections: Only keep cones with high confidence
        cone_detections = detections[
            (detections.confidence > self.confidence_threshold)
            & (detections.class_id == self.cone_class_id)
        ]

        frame_resolution = frame.shape[1] * frame.shape[0]

        # Initialize a variable to track the largest bounding box area
        #largest_area = 0
        smallest_normalized_area=0
        # Iterate through the cone detections to calculate bounding box areas
        for detection in cone_detections:
            normalized_areas = torch.tensor([
                (detection[0][2] * detection[0][3]) / frame_resolution
                for detection in cone_detections
            ]).to(device)
            
            smallest_normalized_area = torch.max(normalized_areas).cpu().item()
        # Check if the largest area exceeds the predefined threshold
        #print(smallest_normalized_area)
        #print(smallest_normalized_area>self.area_threshold)
            print(smallest_normalized_area)
            print(self.area_threshold)
            if (
                smallest_normalized_area >= self.area_threshold  # Replace with your area threshold value
                 and not self.stop_published  # Ensure stop is only published once
             ):
                stop_msg = String()  # Create a String message
                stop_msg.data = "stop"  # Set the message data to "stop"
                self.publisher_.publish(stop_msg)  # Publish the stop message
                self.get_logger().info(f"Stop command published! Area: {smallest_normalized_area}")
                self.stop_published = True  # Set flag to true after publishing

                # Annotate and display the image
        annotated_image = self.bounding_box_annotator.annotate(
                    scene=frame, detections=cone_detections
                )
        annotated_image = self.label_annotator.annotate(
                    scene=annotated_image, detections=cone_detections
                )

            # cv2.imshow('Cone Detection', annotated_image)

            # Display the processed frame
            # fram = cv2.flip(frame, 0)
        cv2.imshow("Detected Arrows", frame)

            # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            return

        # dc.release()
        # cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = DirectionPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()