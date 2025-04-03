#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Dense>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp> 
#include <bits/stdc++.h>
#include <vector>
#include <algorithm>
using namespace std;

class MapPoint : public rclcpp::Node {
public:
    enum obstacleEnum{
        OBSTACLE,
        PIT,
        SAFE
    };

    struct obstacleInfo {
        obstacleEnum obstacle;
        float x;
        float y;
        float z;
        float distance;
    };

    MapPoint() : Node("map_point") {
        point_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 10,
            std::bind(&MapPoint::processPointCloud, this, std::placeholders::_1)
        );

        direction_publisher = this->create_publisher<std_msgs::msg::String>("/pit_direction", 10);
    }

private:
    static constexpr float OBSTACLE_THRESHOLD = 0.75;
    static constexpr float SAFE_DISTANCE = 2;
    static constexpr float ground_up=1.07;
    static constexpr float ground_down=1.5;
    static constexpr float real_height=0.33;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr direction_publisher;

    void preProcessPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cloud);
    }

    vector<obstacleInfo> filterPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
        vector<obstacleInfo> filteredPointCloud;

        for (const auto& point : cloud->points) {
            float distance = sqrt((point.x * point.x) + (point.y * point.y) + (point.z * point.z));
            if (point.z < SAFE_DISTANCE) {
                filteredPointCloud.push_back({SAFE, point.x, point.y, point.z, distance});
            }
        }

        return filteredPointCloud;
    }

    vector<obstacleInfo> pitFilterPointCloud(vector<obstacleInfo>& cloud){
        vector<obstacleInfo> pits;
        for(const auto& point : cloud){
            float distance = sqrt((point.x*point.x) + (point.y*point.y) + (point.z*point.z));
            if(point.y > real_height){
                pits.push_back({PIT, point.x, point.y, point.z, distance});
            }
        }

        return pits;
    }

    vector<obstacleInfo> obstacleFilterPointCloud(vector<obstacleInfo>& cloud){
        vector<obstacleInfo> obstacles;
        for(const auto& point : cloud){
            float distance = sqrt((point.x*point.x) + (point.y*point.y) + (point.z*point.z));
            if(point.y < real_height){
                obstacles.push_back({OBSTACLE, point.x, point.y, point.z, distance});
            }
        }
        return obstacles;
    }
    // obstacleInfo getMinPitObstacle(vector<obstacleInfo>& obstaclePit){
    //     if(obstaclePit.empty()){
    //         RCLCPP_WARN(this->get_logger(), "%s", "Empty");
    //     }
    //     sort(obstaclePit.begin(), obstaclePit.end(), [](const obstacleInfo& a, const obstacleInfo& b){
    //         return a.distance < b.distance;
    //     });
    //     return obstaclePit[0];
    // }

    string getObstacleDirection(vector<obstacleInfo>& obstacles){
        string direction;
        // front_count = 0;
        int front_left_count = 0;
        int front_right_count = 0;
        int left_count = 0;
        int right_count = 0;
        for(const auto& obstacle : obstacles){
            RCLCPP_WARN(this->get_logger(), "Distance=%.2f, Height=%.2f, x_direction=%.2f", obstacle.z, obstacle.y, obstacle.x);
            if(abs(obstacle.x) <= 0.75 && obstacle.x > 0){
                front_right_count++;
            }
            else if(abs(obstacle.x) <= 0.75 && obstacle.x < 0){
                front_left_count++;
            }
            else if(obstacle.x > 0.6){
                right_count++;
            }
            else{
                left_count++;
            }
        }
        if(front_right_count > front_left_count && front_right_count > left_count && front_right_count > right_count){
            direction = "Front_Right";
        }
        else if(front_left_count > front_right_count && front_left_count > left_count && front_left_count > right_count){
            direction = "Front_Left";
        }
        else if(left_count > front_right_count && left_count > front_left_count && left_count > right_count){
            direction = "Left";
        }
        else if(right_count > front_right_count && right_count > front_left_count && right_count > left_count){
            direction = "Right";
        }
        else{
            direction = "Front";
        }

        return direction;
    }

    void drawOnFrame_obj(vector<obstacleInfo>& obstacles) {
        cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);

        float frame_width = frame.cols;
        float frame_height = frame.rows;

        float x_scale = frame_width / 10.0;
        float y_scale = frame_height / 10.0;

        for (const auto& obstacle : obstacles) {
            if (obstacle.y < 100 && obstacle.y > 0.46) {
                int x = static_cast<int>((obstacle.x + 5.0) * x_scale);
                int y = static_cast<int>((obstacle.z ) * y_scale);

                x = std::max(0, std::min(x, static_cast<int>(frame_width - 1)));
                y = std::max(0, std::min(y, static_cast<int>(frame_height - 1)));

                RCLCPP_WARN(this->get_logger(), "Distance=%.2f, Height=%.2f, x_direction=%.2f", obstacle.z, obstacle.y, obstacle.x);


                cv::circle(frame, cv::Point(x, y), 1, cv::Scalar(0, 255, 0), -1);
            }
        }
    // Convert frame to grayscale for contour detection
    // cv::Mat gray;
    // cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    // // Threshold the grayscale image to create a binary image
    // cv::Mat binary;
    // cv::threshold(gray, binary, 1, 255, cv::THRESH_BINARY);

    // // Find contours in the binary image
    // vector<vector<cv::Point>> contours;
    // cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // // Draw contours on the frame
    // cv::drawContours(frame, contours, -1, cv::Scalar(0, 0, 255), 1); // Red contours

    // Display the frame with contours
    cv::imshow("Obstacle Map with Contours", frame);
    cv::waitKey(1);
    }

    // draw on frame for pit  point with inof and find the edge 

//    void drawOnFrame_pit(vector<obstacleInfo>& pits) {
//     cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);

//     float frame_width = frame.cols;
//     float frame_height = frame.rows;

//     float x_scale = frame_width / 10.0;
//     float y_scale = frame_height / 10.0;

//     for (const auto& pit : pits) {
//                int x = static_cast<int>((pit.x + 5.0) * x_scale);
//                int y = static_cast<int>((pit.y) * y_scale);
//                // Clamp the points to fit within the frame
//                x = std::max(0, std::min(x, static_cast<int>(frame_width - 1)));
//                y = std::max(0, std::min(y, static_cast<int>(frame_height - 1)));

//                RCLCPP_ERROR(this->get_logger(), "Distance=%.2f, Height=%.2f, x_direction=%.2f", obstacle.z, obstacle.y, obstacle.x);
//                // Plot the points on the frame;/
//                cv::circle(frame, cv::Point(x, y), 1, cv::Scalar(0, 255, 0), -1);
//         }
// //     cv::Mat gray;
// //     cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
// //     // Threshold the grayscale image to create a binary image
// //     cv::Mat binary;
// //     cv::threshold(gray, binary, 1, 255, cv::THRESH_BINARY);

// //     // Find contours in the binary image
// //     vector<vector<cv::Point>> contours;
// //     cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

// //     // Draw contours on the frame
// //     cv::drawContours(frame, contours, -1, cv::Scalar(0, 0, 255), 1); // Red contours

// //     // Display the frame with contours
//     cv::imshow("Obstacle Map with pits", frame);
//     cv::waitKey(1);
//  }
    

    void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);
        auto message = std_msgs::msg::String();

        preProcessPointCloud(cloud);
        vector<obstacleInfo> filteredPointCloud = filterPointCloud(cloud);
        // vector<obstacleInfo> pits = pitFilterPointCloud(filteredPointCloud);
        vector<obstacleInfo> obstacles = obstacleFilterPointCloud(filteredPointCloud); 
        // if(pit.empty())
        // pits.insert(pits.end(), obstacles.begin(), obstacles.end());
        // obstacleInfo minThing = getMinPitObstacle(pits);
        // obstacleEnum obj = minThing.obstacle
        string direction = getObstacleDirection(obstacles);
        message.data = direction;
        direction_publisher->publish(message);
        // RCLCPP_WARN(this->get_logger(), "Publishing direction: %s", direction.c_str());

        
        // auto pits = pit_filterPointCloud(cloud);
        // for now another draw on frame different cause the plane is gonna be y and x of the realsense
        // drawOnFrame_obj(obstacles);
        //the plane of the drawing is gonna be in the x and z plane of the realsense
        // drawOnFrame_obj(pits);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapPoint>());
    rclcpp::shutdown();
    return 0;
}

// one more idea of the 

// void drawOnFrame_obj(vector<obstacleInfo>& obstacles, vector<obstacleInfo>& pits) {
//     cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);

//     float frame_width = frame.cols;
//     float frame_height = frame.rows;

//     float x_scale = frame_width / 10.0; // Scale for X-axis
//     float y_scale = frame_height / 10.0; // Scale for Z-axis

//     // Draw obstacles in the X-Z plane
//     for (const auto& obstacle : obstacles) {
//         if (obstacle.z > OBSTACLE_THRESHOLD) {
//             int x = static_cast<int>((obstacle.x + 5.0) * x_scale);
//             int y = static_cast<int>((obstacle.z) * y_scale);

//             x = std::max(0, std::min(x, static_cast<int>(frame_width - 1)));
//             y = std::max(0, std::min(y, static_cast<int>(frame_height - 1)));

//             cv::circle(frame, cv::Point(x, y), 1, cv::Scalar(0, 255, 0), -1); // Green for obstacles
//         }
//     }

//     // Draw pits as vertical lines in the X-Z plane
//     for (const auto& pit : pits) {
//         int x = static_cast<int>((pit.x + 5.0) * x_scale);
//         int y_start = static_cast<int>(0); // Start at the bottom of the frame
//         int y_end = static_cast<int>((pit.z) * y_scale); // End at the Z-coordinate of the pit

//         x = std::max(0, std::min(x, static_cast<int>(frame_width - 1)));
//         y_end = std::max(0, std::min(y_end, static_cast<int>(frame_height - 1)));

//         // Draw a vertical line representing the pit
//         cv::line(frame, cv::Point(x, y_start), cv::Point(x, y_end), cv::Scalar(255, 0, 0), 2); // Blue for pits
//     }

//     // Convert frame to grayscale for contour detection
//     cv::Mat gray;
//     cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
//     // Threshold the grayscale image to create a binary image
//     cv::Mat binary;
//     cv::threshold(gray, binary, 1, 255, cv::THRESH_BINARY);

//     // Find contours in the binary image
//     vector<vector<cv::Point>> contours;
//     cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

//     // Draw contours on the frame
//     cv::drawContours(frame, contours, -1, cv::Scalar(0, 0, 255), 1); // Red contours

//     // Display the frame with contours
//     cv::imshow("Obstacle Map with Contours", frame);
//     cv::waitKey(1);
// }



// idea 2 with respect to that of the threshold distance
