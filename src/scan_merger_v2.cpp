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


class ScanMergerV2 : public rclcpp::Node
{
public:
    //consturctor
    ScanMergerV2() : Node("scan_merger_v2"), tf_buffer_(get_clock())
    {

        max_range_ = 12.0f; // Max range of individual LiDARs
        angle_min_ = -M_PI; // Full 360 view for combined scan
        angle_max_ = M_PI;
        angle_increment_ = 0.01f; // Adjust as needed
        num_points_ = static_cast<int>((angle_max_ - angle_min_) / angle_increment_) + 1;

        lidar1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan1",
            10,
            std::bind(&ScanMergerV2::scan1_callback, this, std::placeholders::_1)
        );
        
        lidar2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan2",
            10,
            std::bind(&ScanMergerV2::scan2_callback, this, std::placeholders::_1)
        );

        merged_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "scan",
            10
        );


        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this);


        // scan publisher calling
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ScanMergerV2::scan_publisher, this)
        );

        RCLCPP_INFO(this->get_logger(), "ScanMergerV2 Node has been started.😃");
    }
private:
    void scan1_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void scan2_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    std::vector<std::pair<float, float>> laserscan_to_point(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, std::string lidar_frame);

    void scan_publisher();

    // initializing pointers and variable
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar2_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr merged_scan_;
    // for storing scan message from callback functions
    sensor_msgs::msg::LaserScan::SharedPtr scan1_;
    sensor_msgs::msg::LaserScan::SharedPtr scan2_;

    // for getting rotation matrix and translation matrix between base_link and laser_frame
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr timer_;

    float max_range_;
    float angle_min_;
    float angle_max_;
    float angle_increment_;
    int num_points_;


};


void ScanMergerV2::scan1_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    scan1_ = msg;
}


void ScanMergerV2::scan2_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    scan2_ = msg;
}


// this is member method
std::vector<std::pair<float, float>> ScanMergerV2::laserscan_to_point(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, std::string lidar_frame){
    std::vector<std::pair<float, float>> points;

    try {
        //main code here
        auto now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
            "base_link",
            lidar_frame,
            now,
            rclcpp::Duration::from_seconds(0.1)
        );

        // Extract translation and rotation
        float tx = transform.transform.translation.x;
        float ty = transform.transform.translation.y;
        float tz = transform.transform.translation.z;

        geometry_msgs::msg::Quaternion q = transform.transform.rotation;

        // convert gemery_msgs quaternion to tf2 quaternion
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);

        // Create 3x3 rotation matrix from the quaternion
        tf2::Matrix3x3 rot_matrix(quat);

        int size = scan_msg->ranges.size();
        float r = 0;
        for (int i=0; i<size; i++){
            r = scan_msg->ranges[i];

            if (r < scan_msg->range_min || r > scan_msg->range_max || !std::isfinite(r)){
                continue;
            } else {

                //calculate points in lidar frame
                float angle = scan_msg->angle_min + i * (scan_msg->angle_increment);
                float x_lidar = r * std::cos(angle);
                float y_lidar = r * std::sin(angle);
                float z_lidar = 0.0;

                // transform point to base_link
                tf2::Vector3 p_lidar(x_lidar, y_lidar, z_lidar);
                tf2::Vector3 p_base = rot_matrix * p_lidar + tf2::Vector3(tx, ty, tz);

                // store points and it's polar angle
                float angle_in_base = std::atan2(p_base.getY(), p_base.getX());
                float distance = sqrt(pow(p_base.getX(), 2) + pow(p_base.getY(), 2));

                points.push_back({angle_in_base, distance});
            }
        }

    }
    catch (const tf2::TransformException &e) {
        RCLCPP_WARN(this->get_logger(), "TF error: %s", e.what());
    }

    return points;
}




void ScanMergerV2::scan_publisher(){
    if (!scan1_ || !scan2_){
        return;
    }

    std::vector<std::pair<float, float>> points1 = this->laserscan_to_point(scan1_, "agv/base_link/gpu_lidar_right");
    std::vector<std::pair<float, float>> points2 = this->laserscan_to_point(scan2_, "agv/base_link/gpu_lidar_left");
    std::vector<std::pair<float, float>> all_points = points1;
    all_points.insert(all_points.end(), points2.begin(), points2.end());

    sensor_msgs::msg::LaserScan combined_scan;
    combined_scan.header.stamp = scan1_->header.stamp;
    combined_scan.header.frame_id = "base_link";
    combined_scan.angle_increment = angle_increment_;
    combined_scan.angle_min = angle_min_;
    combined_scan.angle_max = angle_max_;
    combined_scan.range_min = 0.5;
    combined_scan.range_max = max_range_;
    combined_scan.scan_time = 0.1;
    combined_scan.time_increment = 0.0;

    // combined_scan.ranges calculation
    // Initialize ranges array with inf
    combined_scan.ranges.assign(num_points_, std::numeric_limits<float>::infinity());

    // Fill in ranges based on combined points
    for (const auto& [angle, distance] : all_points) {
        int idx = static_cast<int>((angle - angle_min_) / angle_increment_);
        if (idx >= 0 && idx < static_cast<int>(combined_scan.ranges.size())) {
            // If multiple points map to same index, take the closest one
            combined_scan.ranges[idx] = std::min(combined_scan.ranges[idx], distance);
        }
    }

    merged_scan_->publish(combined_scan);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanMergerV2>());
  rclcpp::shutdown();
  return 0;
}