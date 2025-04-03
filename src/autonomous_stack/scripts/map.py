#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import folium
import csv
import os
import threading

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')

        # File paths
        self.gps_file_path = 'gps_data.csv'
        self.direction_file_path = 'direction_data.csv'
        self.gps_data = []  # List to hold GPS data
        self.directions = []  # List to hold direction data
        self.direction_buffer = []  # Buffer for directions

        # Threading lock for synchronization
        self.lock = threading.Lock()

        # Create files and write headers
        self.gps_file = open(self.gps_file_path, 'w', newline='')
        self.direction_file = open(self.direction_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.gps_file)
        self.csv_direction_writer = csv.writer(self.direction_file)
        self.csv_writer.writerow(['Latitude', 'Longitude', 'Altitude'])  # Header for GPS data
        self.csv_direction_writer.writerow(['Latitude', 'Longitude', 'Direction'])  # Header for direction data

        # Subscribers
        self.create_subscription(String, 'gps_data', self.gps_data_callback, 10)
        self.create_subscription(String, 'direction', self.direction_callback, 10)
        self.create_subscription(String, 'stop_command', self.stop_command_callback, 10)

    def gps_data_callback(self, msg):
        data = msg.data.split(',')
        if len(data) >= 3:
            try:
                latitude = float(data[0].split(':')[1].strip())
                longitude = float(data[1].split(':')[1].strip())
                altitude = float(data[2].split(':')[1].strip().replace('m', '').strip())

                with self.lock:
                    self.gps_data.append([latitude, longitude, altitude])
                    self.csv_writer.writerow([latitude, longitude, altitude])
                    self.gps_file.flush()  # Ensure data is written immediately

                    # Process any buffered directions
                    while self.direction_buffer:
                        direction = self.direction_buffer.pop(0)
                        self.process_direction(direction)

                self.get_logger().info(f'Received GPS Data: {latitude}, {longitude}, {altitude}')
            except ValueError:
                self.get_logger().error("Error parsing GPS data")
        else:
            self.get_logger().error("Incorrect data format received")

    def direction_callback(self, msg):
        direction = msg.data.strip().lower()
        if direction in ['left', 'right', 'stop']:
            with self.lock:
                if self.gps_data:
                    self.process_direction(direction)
                else:
                    # Buffer the direction if no GPS data is available
                    self.direction_buffer.append(direction)

    def process_direction(self, direction):
        last_gps_point = self.gps_data[-1]
        self.directions.append([last_gps_point[0], last_gps_point[1], direction])
        self.csv_direction_writer.writerow([last_gps_point[0], last_gps_point[1], direction])
        self.direction_file.flush()
        self.get_logger().info(f"Direction '{direction.capitalize()}' processed at {last_gps_point[0]}, {last_gps_point[1]}")

    def save_plot(self):
        if not self.gps_data:
            self.get_logger().warn("No GPS data to plot.")
            return

        # Create map centered on the first GPS point
        map_center = self.gps_data[0][:2]
        m = folium.Map(location=map_center, zoom_start=18)

        # Add polyline for GPS path
        gps_path = [(data[0], data[1]) for data in self.gps_data]
        folium.PolyLine(gps_path, color="blue", weight=2.5, opacity=1).add_to(m)

        # Add direction markers
        for direction_data in self.directions:
            latitude, longitude, direction = direction_data
            marker_color = 'green' if direction == 'left' else 'red' if direction == 'right' else 'black'
            folium.Marker(
                location=[latitude, longitude],
                popup=f'Direction: {direction.capitalize()}',
                icon=folium.Icon(color=marker_color)
            ).add_to(m)

        # Save HTML map
        html_file = "gps_path.html"
        m.save(html_file)
        self.get_logger().info(f"GPS path saved as HTML at {html_file}")

    def stop_command_callback(self, msg):
        if msg.data.strip().lower() == 'stop':
            self.get_logger().info("Stop command received! Generating final plot and shutting down.")

            with self.lock:
                if self.gps_data:
                    last_gps_point = self.gps_data[-1]
                    self.directions.append([last_gps_point[0], last_gps_point[1], 'stop'])
                    self.csv_direction_writer.writerow([last_gps_point[0], last_gps_point[1], 'stop'])
                    self.direction_file.flush()

            self.gps_file.close()
            self.direction_file.close()
            self.save_plot()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()
    try:
        rclpy.spin(gps_publisher)
    except KeyboardInterrupt:
        gps_publisher.get_logger().info("Node interrupted. Generating final plot...")
    finally:
        with gps_publisher.lock:
            gps_publisher.gps_file.close()
            gps_publisher.direction_file.close()
            gps_publisher.save_plot()
            gps_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
