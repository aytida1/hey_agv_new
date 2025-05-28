#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <mutex>
#include <memory>


class ScanMergerV2 : public rclcpp::Node
{
public:
    //constructor
    ScanMergerV2() : Node("scan_merger_v2"), tf_buffer_(get_clock())
    {
        max_range_ = 12.0f;
        angle_min_ = -M_PI;
        angle_max_ = M_PI;
        angle_increment_ = 0.00348f;
        num_points_ = static_cast<int>((angle_max_ - angle_min_) / angle_increment_) + 1;

        // Set TF buffer cache time to handle past transforms
        tf_buffer_.setUsingDedicatedThread(true);

        lidar1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan1",
            rclcpp::SensorDataQoS(),  // Use sensor data QoS for better real-time performance
            std::bind(&ScanMergerV2::scan1_callback, this, std::placeholders::_1)
        );
        
        lidar2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan2",
            rclcpp::SensorDataQoS(),
            std::bind(&ScanMergerV2::scan2_callback, this, std::placeholders::_1)
        );

        merged_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "scan",
            rclcpp::SensorDataQoS()
        );

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this);

        // Remove timer-based publishing - process immediately when scans arrive
        
        RCLCPP_INFO(this->get_logger(), "ScanMergerV2 Node has been started.😃");
    }

private:
    void scan1_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void scan2_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    std::vector<std::pair<float, float>> laserscan_to_point(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, std::string lidar_frame);
    void process_and_publish_scans();

    // initializing pointers and variables
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar2_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr merged_scan_;
    
    // Store latest scans with thread safety
    sensor_msgs::msg::LaserScan::SharedPtr scan1_;
    sensor_msgs::msg::LaserScan::SharedPtr scan2_;
    std::mutex scan_mutex_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    float max_range_;
    float angle_min_;
    float angle_max_;
    float angle_increment_;
    int num_points_;

    // Track last processed timestamps to avoid duplicate processing
    rclcpp::Time last_scan1_time_;
    rclcpp::Time last_scan2_time_;
};


void ScanMergerV2::scan1_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan1_ = msg;
    last_scan1_time_ = msg->header.stamp;
    
    // Process immediately when scan arrives
    process_and_publish_scans();
}


void ScanMergerV2::scan2_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan2_ = msg;
    last_scan2_time_ = msg->header.stamp;
    
    // Process immediately when scan arrives
    process_and_publish_scans();
}


// this is member method
std::vector<std::pair<float, float>> ScanMergerV2::laserscan_to_point(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, std::string lidar_frame){
    std::vector<std::pair<float, float>> points;

    try {
        // Use the exact scan timestamp with longer timeout for reliability
        geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
            "base_link",
            lidar_frame,
            scan_msg->header.stamp,
            rclcpp::Duration::from_seconds(0.1)  // Increased timeout for better reliability
        );

        // Extract translation and rotation
        float tx = transform.transform.translation.x;
        float ty = transform.transform.translation.y;
        float tz = transform.transform.translation.z;

        geometry_msgs::msg::Quaternion q = transform.transform.rotation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 rot_matrix(quat);

        int size = scan_msg->ranges.size();
        for (int i = 0; i < size; i++){
            float r = scan_msg->ranges[i];

            if (r < scan_msg->range_min || r > scan_msg->range_max || !std::isfinite(r)){
                continue;
            }

            float angle = scan_msg->angle_min + i * (scan_msg->angle_increment);
            float x_lidar = r * std::cos(angle);
            float y_lidar = r * std::sin(angle);
            float z_lidar = 0.0;

            tf2::Vector3 p_lidar(x_lidar, y_lidar, z_lidar);
            tf2::Vector3 p_base = rot_matrix * p_lidar + tf2::Vector3(tx, ty, tz);

            float angle_in_base = std::atan2(p_base.getY(), p_base.getX());
            float distance = sqrt(pow(p_base.getX(), 2) + pow(p_base.getY(), 2));

            points.push_back({angle_in_base, distance});
        }

    }
    catch (const tf2::TransformException &e) {
        // Silently skip if transform not available to avoid lag
        return points;
    }

    return points;
}


void ScanMergerV2::process_and_publish_scans(){
    // Already locked by caller
    // Don't require both scans - publish if we have at least one
    if (!scan1_ && !scan2_){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, "No scan data available");
        return;
    }

    // Use the most recent available timestamp
    rclcpp::Time sync_time = this->get_clock()->now();
    if (scan1_ && scan2_) {
        rclcpp::Time time1(scan1_->header.stamp);
        rclcpp::Time time2(scan2_->header.stamp);
        sync_time = (time1 > time2) ? time1 : time2;
    } else if (scan1_) {
        sync_time = rclcpp::Time(scan1_->header.stamp);
    } else if (scan2_) {
        sync_time = rclcpp::Time(scan2_->header.stamp);
    }

    sync_time = sync_time - rclcpp::Duration::from_seconds(0.0);  // 0.1 seconds = 100ms
    std::vector<std::pair<float, float>> points1, points2;
    
    // Process scan1 if available
    if (scan1_) {
        points1 = this->laserscan_to_point(scan1_, "agv/base_link/gpu_lidar_right");
        if (points1.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000, "Failed to transform scan1");
        }
    }
    
    // Process scan2 if available  
    if (scan2_) {
        points2 = this->laserscan_to_point(scan2_, "agv/base_link/gpu_lidar_left");
        if (points2.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000, "Failed to transform scan2");
        }
    }
    
    // Continue even if one transform failed, as long as we have some points
    if (points1.empty() && points2.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, "No valid transformed points available");
        return;
    }

    // Combine points
    std::vector<std::pair<float, float>> all_points = points1;
    all_points.insert(all_points.end(), points2.begin(), points2.end());

    sensor_msgs::msg::LaserScan combined_scan;
    combined_scan.header.stamp = sync_time;  // Use synchronized timestamp
    combined_scan.header.frame_id = "base_link";
    combined_scan.angle_increment = angle_increment_;
    combined_scan.angle_min = angle_min_;
    combined_scan.angle_max = angle_max_;
    combined_scan.range_min = 0.5;
    combined_scan.range_max = max_range_;
    combined_scan.scan_time = 0.01;  // Faster scan time for reduced latency
    combined_scan.time_increment = 0.0;

    // Initialize ranges array with inf
    combined_scan.ranges.assign(num_points_, std::numeric_limits<float>::infinity());

    // Fill in ranges based on combined points with optimized binning
    for (const auto& [angle, distance] : all_points) {
        // Normalize angle
        float normalized_angle = angle;
        while (normalized_angle < angle_min_) normalized_angle += 2 * M_PI;
        while (normalized_angle > angle_max_) normalized_angle -= 2 * M_PI;
        
        int idx = static_cast<int>((normalized_angle - angle_min_) / angle_increment_);
        if (idx >= 0 && idx < static_cast<int>(combined_scan.ranges.size())) {
            combined_scan.ranges[idx] = std::min(combined_scan.ranges[idx], distance);
        }
    }

    // Publish immediately without any additional delay
    merged_scan_->publish(combined_scan);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanMergerV2>());
  rclcpp::shutdown();
  return 0;
}