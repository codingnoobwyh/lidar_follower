#include "../include/lidar_filter.hpp"
#include <cmath>
#include <rclcpp/rclcpp.hpp>

LidarFilter::LidarFilter()
    : Node("lidar_follower"), minShieldAngle(-M_PI / 4), maxShieldAngle(M_PI / 4), distanceThreshold(2.0), arcResolution(20), angleWrap(false) {
    // 初�?�化订阅和发�?
    scanSub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&LidarFilter::scanCallback, this, std::placeholders::_1));
    shieldSub =
        this->create_subscription<std_msgs::msg::Float64MultiArray>("shield", 10, std::bind(&LidarFilter::angleCallback, this, std::placeholders::_1));

    filteredScanPub =
        this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_scan", 10);
    markerPub =
        this->create_publisher<visualization_msgs::msg::Marker>("shield_range", 10);
    setShieldAngles(minShieldAngle, maxShieldAngle);
}

void LidarFilter::setShieldAngles(float minAngle, float maxAngle) {
    minShieldAngle = normalizeAngle(minAngle);
    maxShieldAngle = normalizeAngle(maxAngle);
    angleWrap      = (maxShieldAngle < minShieldAngle);
}

void LidarFilter::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto         filteredScan = *msg;
    const size_t numPoints    = msg->ranges.size();
    for (size_t i = 0; i < numPoints; ++i) {
        float angle = msg->angle_min + i * msg->angle_increment;
        angle       = normalizeAngle(angle);

        bool inShieldArea = false;
        if (angleWrap) {
            inShieldArea = (angle >= minShieldAngle) || (angle <= maxShieldAngle);
        } else {
            inShieldArea = (angle >= minShieldAngle) && (angle <= maxShieldAngle);
        }
        if (!inShieldArea || msg->ranges[i] > this->distanceThreshold) {
            filteredScan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        }
    }

    filteredScanPub->publish(filteredScan);
    publishShieldRange();
}

void LidarFilter::angleCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 3) {
        RCLCPP_ERROR(this->get_logger(), "wrong format of lidar shield angles");
        return;
    }
    setShieldAngles(msg->data.at(0), msg->data.at(1));
    distanceThreshold = msg->data.at(2);
    RCLCPP_INFO(this->get_logger(), "received %.2f and %.2f\r\nmin shield angle set as %.2f max shield angle set as %.2f",
        msg->data.at(0), msg->data.at(1), minShieldAngle, maxShieldAngle);
}

visualization_msgs::msg::Marker LidarFilter::createMarker(int id, float angle, float range) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "laser_link";
    marker.header.stamp    = this->now();
    marker.ns              = "shield_limits";
    marker.id              = id;
    marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.scale.x         = 0.02;

    geometry_msgs::msg::Point start, end;
    start.x = 0.0;
    start.y = 0.0;
    end.x   = range * cos(angle);
    end.y   = range * sin(angle);

    marker.points.push_back(start);
    marker.points.push_back(end);
    marker.color.a = 1.0;
    marker.color.r = (id == 1);
    marker.color.g = (id == 0);
    marker.color.b = 0.00;
    return marker;
};

geometry_msgs::msg::Point LidarFilter::polarToCartesian(double r, double theta) {
    geometry_msgs::msg::Point p;
    p.x = r * cos(theta);
    p.y = r * sin(theta);
    p.z = 0.0;
    return p;
}

void LidarFilter::publishShieldRange(void) {
    // 创建基�?�Marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "laser_link";
    marker.header.stamp    = this->now();
    marker.ns              = "rotating_sector";
    marker.id              = 0;
    marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.scale.x         = 0.01;  // 线�??
    marker.color.r         = 1.0f;  // 红色
    marker.color.a         = 1.0f;  // 不透明

    // 计算关键�?
    geometry_msgs::msg::Point center;
    center.x = center.y = center.z = 0.0;
    // 添加点序�?
    marker.points.push_back(center);
    // �?一条半�?
    marker.points.push_back(polarToCartesian(distanceThreshold, minShieldAngle));

    auto sectorAngle = maxShieldAngle - minShieldAngle;
    if (angleWrap)
        sectorAngle += 2 * M_PI;

    // 圆弧部分
    for (int i = 0; i <= arcResolution; ++i) {
        double theta = normalizeAngle(minShieldAngle + sectorAngle * i / arcResolution);
        marker.points.push_back(polarToCartesian(distanceThreshold, theta));
    }

    // �?二条半径
    marker.points.push_back(polarToCartesian(distanceThreshold, maxShieldAngle));
    marker.points.push_back(center);

    markerPub->publish(marker);
}

float LidarFilter::normalizeAngle(float angle) {
    angle = fmod(angle, 2 * M_PI);
    if (angle < 0)
        angle += 2 * M_PI;
    return angle;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::Rate loopRate(50);
    auto         node = std::make_shared<LidarFilter>();
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        loopRate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
