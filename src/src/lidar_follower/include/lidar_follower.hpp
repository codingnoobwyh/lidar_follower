#ifndef LIDAR_FOLLOWER_HPP
#define LIDAR_FOLLOWER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

class LidarFollower : public rclcpp::Node {
  public:
    LidarFollower();
    std::vector<std::pair<double, double>> trackedLegs;

  private:
    struct Point {
        float x;
        float y;
        Point(float x = 0, float y = 0) : x(x), y(y) {}
    };

    struct Cluster {
        std::vector<Point> points;
        Point              centroid;
        float              radius;
        float              centroidAngle;
    };
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr shieldPub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        cmdVelPub;

    // Parameters
    float clusterThreshold;
    int   minLegPoints;
    float maxLegRadius;
    float legPairDistance;
    float targetAngle;
    int   maxLegsToTrack;
    float angleRedundancy;
    float distRedundancy;

    struct PID {
        double kp;
        double ki;
        double kd;
        double integral;
        double prevError;
    } anglePID, distPID;
    float        pidTargetDist;
    float        pidTargetAngle;
    rclcpp::Time lastTime;


    void stop();
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    std::vector<Point> convertScanToPoints(const sensor_msgs::msg::LaserScan& scan);
    std::vector<Cluster> performClustering(const std::vector<Point>& points);
    Cluster              analyzeCluster(const Cluster& cluster);
    std::vector<Cluster> filterClusters(const std::vector<Cluster>& clusters);
    void                 findLegPairs(const std::vector<Cluster>& legs);
    visualization_msgs::msg::Marker createLegMarker(const Point& center, int id);
    float  normalizeAngle(float angle);
    double pidUpdate(PID& pid, double error, double dt);
};

#endif