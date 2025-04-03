#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <vector>
#include <string>
#include <cmath>
#include <std_msgs/msg/string.hpp>

using namespace std;

class Pitpoint : public rclcpp::Node {
public:
    enum ObstacleEnum {
        OBSTACLE,
        PIT,
        SAFE
    };

    struct ObstacleInfo {
        float x;
        float y;
        float z;
        float distance;
    };

    Pitpoint() : Node("pit_point") {
        point_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 10,
            std::bind(&Pitpoint::processPointCloud, this, std::placeholders::_1)
        );

        direction_publisher = this->create_publisher<std_msgs::msg::String>("/pit_direction", 10);
    }

private:
    static constexpr float SAFE_DISTANCE = 3.0;
    static constexpr float HORY_DIST = 0.75;
    static constexpr float REAL_HEIGHT = 0.8;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr direction_publisher;

    void preProcessPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cloud);
    }

    vector<ObstacleInfo> filterPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
        vector<ObstacleInfo> filteredPointCloud;

        for (const auto& point : cloud->points) {
            float distance = sqrt((point.x * point.x) + (point.y * point.y) + (point.z * point.z));
            if (point.z < SAFE_DISTANCE) {
                filteredPointCloud.push_back({point.x, point.y, point.z, distance});
            }
        }
        return filteredPointCloud;
    }

    vector<ObstacleInfo> pitFilterPointCloud(vector<ObstacleInfo>& cloud) {
        vector<ObstacleInfo> pits;
        for (const auto& point : cloud) {
            if (point.y > REAL_HEIGHT && abs(point.x) < HORY_DIST) {
                pits.push_back({point.x, point.y, point.z, point.distance});
            }
        }
        return pits;
    }

    string getDirection(const vector<ObstacleInfo>& pits) {
        string direction;
        float sum_x = 0;

        for (const auto& pit : pits) {
            RCLCPP_WARN(this->get_logger(), "Distance=%.2f, Height=%.2f, X_Direction=%.2f", pit.z, pit.y, pit.x);
            sum_x += pit.x;
        }

        if (!pits.empty()) {
            if (sum_x >= 0) {
                direction = "Front_Right";
            } else {
                direction = "Front_Left";
            }
        } else {
            direction = "Empty";
        }
        return direction;
    }

    void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        auto message = std_msgs::msg::String();
        preProcessPointCloud(cloud);

        vector<ObstacleInfo> filteredPointCloud = filterPointCloud(cloud);
        vector<ObstacleInfo> pits = pitFilterPointCloud(filteredPointCloud);

        string direction = getDirection(pits);
        message.data = direction;

        direction_publisher->publish(message);
        RCLCPP_WARN(this->get_logger(), "Publishing direction: %s", direction.c_str());
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pitpoint>());
    rclcpp::shutdown();
    return 0;
}
