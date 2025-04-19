#include <cmath>
#include <algorithm>
#include <cfloat>
#include "../include/lidar_follower.hpp"

LidarFollower::LidarFollower() : Node("lidar_follower"), 
                                 clusterThreshold(0.15f),
                                 minLegPoints(3),
                                 maxLegRadius(0.2f),
                                 legPairDistance(0.3f),
                                 targetAngle(0.0f),
                                 maxLegsToTrack(2),
                                 angleRedundancy(10.0f / 180 * M_PI),
                                 distRedundancy(0.3f), 
                                 anglePID({0.1f, 0.0f, 0.0f, 0.0f, 0.0f}), 
                                 distPID({0.1f, 0.0f, 0.0f, 0.0f, 0.0f}),
                                 pidTargetDist(0.5f), 
                                 pidTargetAngle(0.0f / 180 * M_PI),
                                 lastTime(this->now())  {

    // 订阅激光雷达数据
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/filtered_scan", 10,
        std::bind(&LidarFollower::scanCallback, this, std::placeholders::_1));

    // 发布可视化标记
    markerPub = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/leg_markers", 10);
    shieldPub = create_publisher<std_msgs::msg::Float64MultiArray>("/shield", 10);
    cmdVelPub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

double LidarFollower::pidUpdate(PID& pid, double error, double dt) {
    pid.integral += error * dt;
    double derivative = (error - pid.prevError) / dt;
    double output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
    pid.prevError = error;
    return output;
}

void LidarFollower::stop() {
    geometry_msgs::msg::Twist cmdVel;
    cmdVel.linear.x  = 0;
    cmdVel.angular.z = 0;
    cmdVelPub->publish(cmdVel);
}

void LidarFollower::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    auto points = convertScanToPoints(*scan);
    auto clusters = performClustering(points);
    auto legClusters = filterClusters(clusters);
    findLegPairs(legClusters);
}

std::vector<LidarFollower::Point> LidarFollower::convertScanToPoints(
  const sensor_msgs::msg::LaserScan& scan) 
{
    std::vector<Point> points;
    const size_t numPoints = scan.ranges.size();

    for (size_t i = 0; i < numPoints; ++i) {
        const float range = scan.ranges[i];
        if (std::isinf(range) || std::isnan(range)) continue;

        const float angle = scan.angle_min + i * scan.angle_increment;
        points.emplace_back(range * cos(angle), range * sin(angle));
    }
    return points;
}

std::vector<LidarFollower::Cluster> LidarFollower::performClustering(
  const std::vector<Point>& points) 
{
    std::vector<Cluster> clusters;
    if (points.empty()) return clusters;

    Cluster currentCluster;
    currentCluster.points.push_back(points[0]);

    for (size_t i = 1; i < points.size(); ++i) {
        const float dx = points[i].x - points[i-1].x;
        const float dy = points[i].y - points[i-1].y;
        const float distance = std::hypot(dx, dy);

        if (distance > clusterThreshold) {
        if ((int)currentCluster.points.size() >= minLegPoints) {
            clusters.push_back(analyzeCluster(currentCluster));
        }
        currentCluster.points.clear();
        }
        currentCluster.points.push_back(points[i]);
    }

    if (!currentCluster.points.empty()) {
        clusters.push_back(analyzeCluster(currentCluster));
    }
    return clusters;
}

LidarFollower::Cluster LidarFollower::analyzeCluster(const Cluster& cluster) {
    Cluster analyzed = cluster;
    
    // 计算质心
    float sumX = 0, sumY = 0;
    for (const auto& p : cluster.points) {
        sumX += p.x;
        sumY += p.y;
    }
    analyzed.centroid = Point(
        sumX / cluster.points.size(),
        sumY / cluster.points.size()
    );

    // 计算最大半径
    float maxDist = 0;
    for (const auto& p : cluster.points) {
        const float dx = p.x - analyzed.centroid.x;
        const float dy = p.y - analyzed.centroid.y;
        maxDist = std::max(maxDist, std::hypot(dx, dy));
    }
    analyzed.radius = maxDist;

    // 预计算质心角度
    analyzed.centroidAngle = atan2(analyzed.centroid.y, analyzed.centroid.x);

    return analyzed;
}

std::vector<LidarFollower::Cluster> LidarFollower::filterClusters(
  const std::vector<Cluster>& clusters) 
{
    std::vector<Cluster> legs;
    for (const auto& cluster : clusters) {
        if (cluster.radius <= maxLegRadius && 
            (int)cluster.points.size() >= minLegPoints) {
        legs.push_back(cluster);
        }
    }
    return legs;
}

void LidarFollower::findLegPairs(const std::vector<Cluster>& legs) {
  visualization_msgs::msg::MarkerArray markers;

    // 角度差排序结构
    struct LegPriority {
        const Cluster* cluster;
        float angleDiff;
        bool operator<(const LegPriority& other) const {
        return angleDiff < other.angleDiff;
        }
    };

    // 计算并排序角度差
    std::vector<LegPriority> legPriorities;
    for (const auto& leg : legs) {
        float diff = fabs(normalizeAngle(leg.centroidAngle - targetAngle));
        legPriorities.push_back({&leg, diff});
    }
    std::sort(legPriorities.begin(), legPriorities.end());

    float avgDist = 0.0f, minAngle = FLT_MAX, maxAngle = FLT_MIN;

    // 选择最佳目标
    int validCount = std::min((int)legPriorities.size(), maxLegsToTrack);
    if (validCount == 0) {
        stop();
        return;
    }
    for (int i = 0; i < validCount; ++i) {
        const auto& leg = *legPriorities[i].cluster;
        auto dist = hypot(leg.centroid.x, leg.centroid.y);
        avgDist += (float)dist;
        minAngle = std::min(minAngle, leg.centroidAngle);
        maxAngle = std::max(maxAngle, leg.centroidAngle);
        
        // 创建圆柱标记
        auto marker = createLegMarker(leg.centroid, i*2);
        marker.color.g = static_cast<float>(i)/validCount;  // 颜色渐变
        markers.markers.push_back(marker);

        // 添加角度差文本
        visualization_msgs::msg::Marker textMarker;
        textMarker.header.frame_id = "laser_link";
        textMarker.header.stamp = now();
        textMarker.ns = "angle_info";
        textMarker.id = i * 2+1;
        textMarker.pose.position.x = leg.centroid.x + 0.15;
        textMarker.pose.position.y = leg.centroid.y;
        textMarker.pose.position.z = 0.3;
        textMarker.scale.z = 0.12;
        textMarker.color.a = 1.0;
        textMarker.color.r = 1.0;
        textMarker.color.g = 1.0;
        markers.markers.push_back(textMarker);
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data.push_back(minAngle - angleRedundancy);
    msg.data.push_back(maxAngle + angleRedundancy);
    msg.data.push_back(avgDist / validCount + distRedundancy);
    shieldPub->publish(msg);

    markerPub->publish(markers);
    rclcpp::Time currentTime = now();
    double dt = (currentTime - lastTime).seconds();
    lastTime = currentTime;
    geometry_msgs::msg::Twist cmdVel;
    RCLCPP_INFO(this->get_logger(), "avg distance: %.3f", avgDist);
    cmdVel.linear.y = std::clamp(pidUpdate(distPID, avgDist - pidTargetDist, dt), -0.05, 0.05);
    cmdVel.angular.z = std::clamp(pidUpdate(anglePID, (minAngle + maxAngle) / 2 - pidTargetAngle, dt), -0.05, 0.05);
    cmdVelPub->publish(cmdVel);
}

float LidarFollower::normalizeAngle(float angle) {
    angle = fmod(angle + M_PI, 2*M_PI);
    return (angle < 0) ? angle + 2*M_PI : angle - M_PI;
}

visualization_msgs::msg::Marker LidarFollower::createLegMarker(
  const Point& center, int id) 
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "laser_link";
    marker.header.stamp = now();
    marker.ns = "legs";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = center.x;
    marker.pose.position.y = center.y;
    marker.pose.position.z = 0;
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.6 * std::min(hypot(center.x, center.y), (double)2.0f);
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.a = 0.7;
    return marker;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarFollower>());
    rclcpp::shutdown();
    return 0;
}