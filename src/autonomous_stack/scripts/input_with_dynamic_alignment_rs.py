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
import subprocess

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
from message_filters import Subscriber, ApproximateTimeSynchronizer

cv2.setUseOptimized(True)

# Constants initialisations for OpenCVeo
dc = DepthCamera()
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
        # objects_bbs_ids = []
        # for rect in objects_rect:
        #     x, y, w, h, contour, direction = rect
        #     cx = (x + x + w) // 2
        #     cy = (y + y + h) // 2
        #     centroid = (cx, cy)

        #     same_object_detected = False
        #     for id, pt in self.center_points.items():
        #         dist = math.hypot(cx - pt[0], cy - pt[1])

        #         if dist < 10:
        #             self.center_points[id] = (cx, cy)
        #             objects_bbs_ids.append(
        #                 [x, y, w, h, id, contour, centroid, direction]
        #             )
        #             same_object_detected = True
        #             break

        #     if not same_object_detected:
        #         self.center_points[self.id_count] = (cx, cy)
        #         objects_bbs_ids.append(
        #             [x, y, w, h, self.id_count, contour, centroid, direction]
        #         )
        #         self.id_count += 1

        # new_center_points = {}
        # for obj_bb_id in objects_bbs_ids:
        #     object_id = obj_bb_id[4]
        #     center = self.center_points[object_id]
        #     new_center_points[object_id] = center
        
        # self.center_points = new_center_points.copy()
        return objects_rect

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
        # self.height=1920
        # self.width=1080
        # initialising publishers and creating a timer to call send_cmd_vel function
        self.direction_publisher = self.create_publisher(String, "direction", 10)
        self.distance_threshold = self.create_publisher(Int32, "distance", 10)
        self.publisher_ = self.create_publisher(String, "/stop_command", 10)
        self.align_publisher = self.create_publisher(Twist, "/align_publisher", 10)
        self.create_timer(1, self.send_cmd_vel)
        self.get_logger().info("Direction Publisher node Chalu")
        # getting the frames from the subscribing to image raw cv2_bridge
        #self.color_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        #self.depth_sub = Subscriber(self, Image, '/camera/camera/depth/image_rect_raw')
        #self.sync = ApproximateTimeSynchronizer(
        #    [self.color_sub, self.depth_sub], queue_size=10, slop=0.1, allow_headerless=True
        #)
        #self.sync.registerCallback(self.synced_callback)
        # self.bridge = CvBridge()
        self.ret=None
        self.frame=None
        # initialising some important variables
        self.prev_direction = None
        self.capture_frame=0
        self.distance_published = False
        self.direction_published = False
        self.stop_published = False
        self.time_delay = 0.0
        self.last_distance_publish_time = None
        self.model = YOLO("/home/pradheep/integrated_ws_final/integrated_autonomous_stack/src/autonomous_stack/scripts/best_new_new.pt")
        self.model.to(device)  # this is the path to the weight file
        self.bounding_box_annotator = sv.BoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()
        self.cone_class_id = 0
        self.confidence_threshold = 0.8
        self.distance_ll=50
        self.distance_ul=150
        #self.FOCAL_LENGTH = 75
        self.scaling_factor = 100 / 1080
        self.REAL_ARROW_WIDTH = 30
        self.REAL_ARROW_HEIGHT =20
        self.border_size = 20
        # self.video_port="/dev/video0"
        #self.arrow_detected = False  # Add this to the initialization of your class
        self.d_frame=None
        # initilising messages to send in topic
        self.vel_msg = Twist()
        self.align_vel_msg = Twist()
        self.latest_color_msg = None
        self.latest_depth_msg = None
        # self.capture= cv2.VideoCapture(self.video_port)
        # self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.height)
        # self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.width)
        # self.capture.set(cv2.CAP_PROP_FPS, 30)  # Higher frame rate



        # if not self.capture.isOpened():
            # raise Exception("Could not open video capture device")
        # self.timer = self.create_timer(0.1, self.process_image)
        self.area_threshold=0.4
        
    """ def synced_callback(self, color_msg, depth_msg):
        self.frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        self.d_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')

        self.frame = cv2.resize(self.frame, (1280, 720))
        self.d_frame = cv2.resize(self.d_frame, (1280, 720))

        self.process_image() """
        
    # def process_image(self):
    #      self.ret, self.frame = self.capture.read()
    #      if self.ret is not None:
    #         # self.capture_frame += 1
    #         # if self.capture_frame % 3 == 0:
    #         self.send_cmd_vel(self.ret,self.frame)
    #      else:
    #         print("Error: Could not read frame from webcam.")

   
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
    

    def control_turtlebot(self, contour, relative_position, distance):
        # Clear velocity message before setting new values
        print("This is the alignment index", relative_position[0])
        self.align_vel_msg.linear.x = 0.0
        self.align_vel_msg.angular.z = 0.0

        # Get the dimensions of the frame
        # height, width, _ = self.frame.shape

        # Define the color and thickness for the lines
        color = (0, 255, 0)  # Green color in BGR
        thickness = 2        # Line thickness

        # Draw vertical lines based on distance intervals
        if 250 < distance <= 400:
            left_limit = -230
            right_limit = 230
        # elif 200 < distance <= 300:
        #     left_limit = -200
        #     right_limit = 200
        elif 140 < distance <= 250:
            left_limit = -150
            right_limit = 150
        else:
            left_limit = None
            right_limit = None

        # Draw the lines if limits are defined
        if left_limit is not None and right_limit is not None:
            left_line_x = int(width / 2 + left_limit)
            right_line_x = int(width / 2 + right_limit)

            left_start_point = (left_line_x, 0)
            left_end_point = (left_line_x, height)
            right_start_point = (right_line_x, 0)
            right_end_point = (right_line_x, height)

            cv2.line(self.frame, left_start_point, left_end_point, color, thickness)
            cv2.line(self.frame, right_start_point, right_end_point, color, thickness)

        # Set angular velocity based on relative position (left/right)
        if relative_position[0]>-320 and relative_position[0]<320:
            if 250 < distance <= 500:
                if relative_position[0] < -250:  # Move left
                    self.align_vel_msg.angular.z = 0.63
                    self.get_logger().info(f"{self.align_vel_msg}")
                    self.align_publisher.publish(self.align_vel_msg)
                elif relative_position[0] > 250:  # Move right
                    self.align_vel_msg.angular.z = -0.63
                    self.get_logger().info(f"{self.align_vel_msg}")
                    self.align_publisher.publish(self.align_vel_msg)
                else:
                    self.align_vel_msg.angular.z = 0.0
                    self.align_vel_msg.linear.x = 1.0
                    self.get_logger().info(f"{self.align_vel_msg}")
                    self.align_publisher.publish(self.align_vel_msg)

          

            elif 140 < distance <= 250:
                if relative_position[0] < -150:  # Move left
                    self.align_vel_msg.angular.z = 0.63
                    self.get_logger().info(f"{self.align_vel_msg}")
                    self.align_publisher.publish(self.align_vel_msg)
                elif relative_position[0] > 150:  # Move right
                    self.align_vel_msg.angular.z = -0.63
                    self.get_logger().info(f"{self.align_vel_msg}")
                    self.align_publisher.publish(self.align_vel_msg)
                else:
                    self.align_vel_msg.angular.z = 0.0
                    self.align_vel_msg.linear.x = 1.0
                    self.get_logger().info(f"{self.align_vel_msg}")
                    self.align_publisher.publish(self.align_vel_msg)

            else:
                self.align_vel_msg.linear.x = 0.0
                self.align_vel_msg.angular.z = 0.0
        else:
            if 250 < distance <= 400:
                if relative_position[0] < -250:  # Move left
                    self.align_vel_msg.angular.z = 0.63
                    self.get_logger().info(f"{self.align_vel_msg}")
                    self.align_publisher.publish(self.align_vel_msg)
                elif relative_position[0] > 250:  # Move right
                    self.align_vel_msg.angular.z = -0.63
                    self.get_logger().info(f"{self.align_vel_msg}")
                    self.align_publisher.publish(self.align_vel_msg)
                else:
                    self.align_vel_msg.angular.z = 0.0
                    self.align_vel_msg.linear.x = 1.0
                    self.get_logger().info(f"{self.align_vel_msg}")
                    self.align_publisher.publish(self.align_vel_msg)

           

            elif 140 < distance <= 250:
                if relative_position[0] < -150:  # Move left
                    self.align_vel_msg.angular.z = 0.63
                    self.get_logger().info(f"{self.align_vel_msg}")
                    self.align_publisher.publish(self.align_vel_msg)
                elif relative_position[0] > 150:  # Move right
                    self.align_vel_msg.angular.z = -0.63
                    self.get_logger().info(f"{self.align_vel_msg}")
                    self.align_publisher.publish(self.align_vel_msg)
                else:
                    self.align_vel_msg.angular.z = 0.0
                    self.align_vel_msg.linear.x = 1.0
                    self.get_logger().info(f"{self.align_vel_msg}")
                    self.align_publisher.publish(self.align_vel_msg)

            else:
                self.align_vel_msg.linear.x = 0.0
                self.align_vel_msg.angular.z = 0.0

    def calculate_brightness(self,roi):
        """
        Calculates the average brightness of a region of interest (ROI).
        
        Args:
            roi (numpy array): ROI in BGR format.

        Returns:
            float: Average brightness value (0-255).
        """
        # Convert the ROI to grayscale
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        # Calculate average brightness
        avg_brightness = np.mean(gray_roi)
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
        ratios = {}
        hist_split = (100, 255) if brightness_threshold < 130 else (150, 255)

        for i, channel_name in enumerate(['Blue', 'Green', 'Red']):
            channel = image[:, :, i]
            hist = cv2.calcHist([channel], [0], None, [256], [0, 256]).flatten()

            hist_low = hist[:hist_split[0] + 1]
            hist_high = hist[hist_split[0] + 1:hist_split[1] + 1]

            global_max_low = max(hist_low)
            global_max_high = max(hist_high)

            ratio = global_max_low / global_max_high if global_max_high != 0 else 0
            ratios[channel_name] = ratio

        ratio_values = list(ratios.values())
        mean_ratio = np.mean(ratio_values)
        variance = np.var(ratio_values)

        return variance * 100, mean_ratio   


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

    def send_cmd_vel(self):
        # with open('/home/pradheep/integrated_ws_final/integrated_autonomous_stack/src/autonomous_stack/scripts/cameraMatrix.pkl', 'rb') as f:
        #     camMatrix = pickle.load(f)
       
        # tracker = EuclideanDistTracker()
       

    # Rest of the processing logic from send_cmd_vel
        msg1 = String()
        msg2 = Int32()
        # tracker = EuclideanDistTracker()
    
            # for RealSense
        ret, d_frame, frame = dc.get_frame()
            # for img_msg to cv2
            # ret, frame = dc.read()
        if not ret:
            print("Error: Could not read frame from webcam.")
            return
        frame_height, frame_width, _ = frame.shape
        frame_center_x = frame_width // 2
        #frame_center_y = frame_height // 2
            #if not ret:
        """ if frame is None and d_frame is None:
                print("Error: Could not read frame from webcam.")
                return """
        #frame_with_border = cv2.UMat(frame)
        frame_with_border = frame
            # Apply median blur (stronger than Gaussian for denoising)
        gray = cv2.cvtColor(frame_with_border, cv2.COLOR_BGR2GRAY)
            # Apply median blur (stronger than Gaussian for denoising)
        frame_denoised = cv2.medianBlur(gray, median_blur_ksize)

            # Apply Gaussian blur
        frame_denoised = cv2.GaussianBlur(frame_denoised, gaussian_blur_ksize, 0)


            # Apply adaptive thresholding
        binary = cv2.adaptiveThreshold(
                frame_denoised, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2
            )

            # Detect edges using Canny
        edges = cv2.Canny(binary, canny_threshold1, canny_threshold2)

            # Find contours
        contours, _ = cv2.findContours(
                edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

         # Detect arrows and calculate tip
        detected_arrows = []
        for contour in contours:
            # # Filter small contours
            # if cv2.contourArea(contour) < 500:  # Adjust threshold if needed
            #     continue
            # Get the bounding rectangle of the contour
            # Get the bounding rectangle of the contour
            x, y, w, h = cv2.boundingRect(contour)

            # Calculate the area manually
            area = w * h

            # Check if the calculated area meets the threshold
            if area < 1000:  # Adjust threshold if needed
                continue
            # Approximate the contour to a polygon
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) == 7:
                angles = self.calculate_angles(approx)
                angles.sort()
                tip_angle1, tip_angle2, tip_angle3 = angles[0], angles[1], angles[2]
                base_angle1, base_angle2 = angles[3], angles[4]
                head_angle1,head_angle2=angles[5],angles[6]
                if (
                    30 <= tip_angle1 <= 90 and
                    70 <= base_angle1 <= 140 and
                    70 <= base_angle2 <= 140 and
                    30 <= tip_angle2 <= 90 and
                    30 <= tip_angle3 <= 90 and
                    70 <= head_angle1 <= 140 and
                    70 <= head_angle2 <= 140 and
                    170 <= (tip_angle1+tip_angle2+tip_angle3)<=200 and
                    350<= (base_angle1+base_angle2+head_angle1+head_angle2)<=480 and
                    170<=(base_angle1+base_angle2)<=200
                ):
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        # Compute the centroid of the contour
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        centroid = (cx, cy)

                        # Compute the bounding box around the contour
                        x, y, w, h = cv2.boundingRect(contour)
                        
                        #roi = frame_with_border[y:y + h, x:x + w]

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
                        #brightness = self.calculate_brightness(roi)
                        #variance, mean_ratio = self.calculate_global_maxima_ratio_split(roi, brightness)

                        #if variance != 0 and mean_ratio != 0 and variance < 3 and mean_ratio < 1.5:
                        detected_arrows.append((x, y, w, h, contour, direction_finalised))
                # global boxes_ids
        # Initialize persistent storage for the closest arrow based on area
        if not hasattr(self, 'persistent_closest_arrow'):
            self.persistent_closest_arrow = None
            self.missed_frames = 0

        # Filter to keep only the arrow with the largest bounding box area
        closest_arrow = None
        largest_area = 0
        for arrow in detected_arrows:
            x, y, w, h, contour, direction_finalised = arrow
            area = w * h  # Calculate the area of the bounding box
            if area > largest_area:
                largest_area = area
                closest_arrow = arrow

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
        # if closest_arrow:
        #     detected_arrows = tracker.update([closest_arrow])
        # else:
        #     detected_arrows = tracker.update([])  # No detection, clear the tracker

        #boxes_ids = tracker.update(detected_arrows)
        distance = 0
        for arrow in detected_arrows:
            x, y, w, h, contour, direction_finalised = arrow
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            mid_x = x + w // 2
            mid_y = y + h // 2
            cv2.circle(frame, (mid_x, mid_y), 10, (0, 0, 255), -1)
            cv2.putText(
                frame,
                direction_finalised,
                (mid_x, mid_y + 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 0, 0),
                2,
            )
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2
            centroid = (cx, cy)
            relative_position = (
                    centroid[0] - frame_center_x,
                    centroid[1] - frame_height // 2,
                )
            distance = self.arrow_distance_estimation(ret, d_frame, centroid[0], centroid[1])
            self.control_turtlebot(contour, relative_position, distance)
            # Display the arrow's data
            self.get_logger().info(f"Distance without alignment is {distance:.3f}")
            cv2.putText(
                frame,
                f"{distance:.3f}mm",
                (mid_x, mid_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 255),
                2,
            )
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)


        if not self.distance_published and self.distance_ll <= distance <= self.distance_ul:
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
                msg1.data = direction_finalised
                self.direction_publisher.publish(msg=msg1)
                self.get_logger().info(f"Published direction: {direction_finalised}")
                self.direction_published = True

        if distance < self.distance_ll or distance > self.distance_ul:
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

        if (
                len(cone_detections) > 0 and not self.stop_published
            ):  # If cones are detected and stop hasn't been published yet
                stop_msg = String()  # Create a String message
                stop_msg.data = "stop"  # Set the message data to "stop"
                self.publisher_.publish(stop_msg)  # Publish the stop message
                self.get_logger().info("Stop command published!")
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
        # results = self.model.predict(source=frame, device=device, verbose=False)[0]
        # detections = sv.Detections.from_ultralytics(results)

        # # Filter detections: Only keep cones with high confidence
        # cone_detections = detections[
        #     (detections.confidence > self.confidence_threshold)
        #     & (detections.class_id == self.cone_class_id)
        # ]

        # # frame_resolution = frame.shape[1] * frame.shape[0]

        # # # Initialize a variable to track the largest bounding box area
        # # #largest_area = 0
        # # smallest_normalized_area=0
        # # # Iterate through the cone detections to calculate bounding box areas
        # # for detection in cone_detections:
        # #     x, y, w, h = detection[0] # Extract bounding box coordinates
        # #     area = w * h  # Calculate the area of the bounding box
        # #     normalized_area = area / frame_resolution
        # #     if normalized_area > smallest_normalized_area:
        # #         smallest_normalized_area = normalized_area
        # # # Check if the largest area exceeds the predefined threshold
        # # #print(smallest_normalized_area)
        # # #print(smallest_normalized_area>self.area_threshold)
        # #     #print(smallest_normalized_area)
        # #     #print(self.area_threshold)
        # #     if (
        # #         smallest_normalized_area >= self.area_threshold  # Replace with your area threshold value
        # #          and not self.stop_published  # Ensure stop is only published once
        # #      ):
        # #         stop_msg = String()  # Create a String message
        # #         stop_msg.data = "stop"  # Set the message data to "stop"
        # #         self.publisher_.publish(stop_msg)  # Publish the stop message
        # #         self.get_logger().info(f"Stop command published! Area: {normalized_area}")
        # #         self.stop_published = True  # Set flag to true after publishing

        # #         # Annotate and display the image
        # # annotated_image = self.bounding_box_annotator.annotate(
        # #             scene=frame, detections=cone_detections
        # #         )
        # # annotated_image = self.label_annotator.annotate(
        # #             scene=annotated_image, detections=cone_detections
        # #         )

        # #     # cv2.imshow('Cone Detection', annotated_image)

        #     # Display the processed frame
        #     # fram = cv2.flip(frame, 0)
        # for detection in cone_detections
        #     stop_msg = String()  # Create a String message
        #     stop_msg.data = "stop"  # Set the message data to "stop"
        #     self.publisher_.publish(stop_msg)  # Publish the stop message
        #     self.get_logger().info(f"Stop command published!")
        #     self.stop_published = True  # Set flag to true after publishing
        #     # Annotate and display the image
        #     annotated_image = self.bounding_box_annotator.annotate(
        #     scene=frame, detections=cone_detections
        #     )
        #     annotated_image = self.label_annotator.annotate(
        #     scene=annotated_image, detections=cone_detections
        #     )
        # cv2.imshow("Detected Arrows", frame)
        

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