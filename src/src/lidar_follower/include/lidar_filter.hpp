#ifndef LIDAR_FILTER_HPP
#define LIDAR_FILTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class LidarFilter : public rclcpp::Node {
  public:
    LidarFilter();

    void  setShieldAngles(float minAngle, float maxAngle);
    float minShieldAngle;
    float maxShieldAngle;
    float distanceThreshold;
    int   arcResolution;

  private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void angleCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    visualization_msgs::msg::Marker createMarker(int id, float angle, float range);
    geometry_msgs::msg::Point polarToCartesian(double r, double theta);
    void                      publishShieldRange(void);
    float                     normalizeAngle(float angle);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr      scanSub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr shieldSub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filteredScanPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub;

    bool angleWrap;
};

#endif