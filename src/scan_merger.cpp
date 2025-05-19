#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <cmath>
#include <iostream>
#include <vector>
#include <array>

class scanMerger : public rclcpp::Node
{
public:
    //constructor
    scanMerger() : Node("ros2_laser_scan_merger")
    {
        yaw_lidar1 = 0.0;
        yaw_lidar2 = 0.0;
        x_lidar1 = 0.115;
        x_lidar2 = -0.115;
        y_lidar1 = -0.16789;
        y_lidar2 = 0.16789;

        laser1_ = std::make_shared<sensor_msgs::msg::LaserScan>();
        laser2_ = std::make_shared<sensor_msgs::msg::LaserScan>();

        rclcpp::QoS qos_profile(10);
        qos_profile.reliable();  // Set reliability to RELIABLE

        //create subcriber of scan1 and scan2 ros2 topics
        sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>("test_lidar", qos_profile, 
            std::bind(&scanMerger::scan1_callback, this, std::placeholders::_1));
        sub2_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan2", qos_profile, 
            std::bind(&scanMerger::scan2_callback, this, std::placeholders::_1));

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos_profile);

        circle_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("test_lidar", qos_profile);
        
        // Timer for publishing circle at 10Hz
        circle_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(90),  // 10Hz
            std::bind(&scanMerger::publish_circle, this));

        RCLCPP_INFO(rclcpp::get_logger("scanMerger"), "Scan Merger Node started👍!");
    }

private:

    void publish_circle()
        {
            sensor_msgs::msg::LaserScan circle_scan;
            circle_scan.header.stamp = this->get_clock()->now();
            circle_scan.header.frame_id = "agv/base_link/gpu_lidar1";
            
            // Define circle parameters
            const int num_points = 200;
            const float radius = 5.0;  // 5 meter radius
            
            // Configure LaserScan message
            circle_scan.angle_min = -M_PI;
            circle_scan.angle_max = M_PI;  // Full circle (360 degrees)
            circle_scan.angle_increment = (2.0 * M_PI) / num_points;
            circle_scan.time_increment = 0.0;
            circle_scan.scan_time = 0.1;  // 10Hz
            circle_scan.range_min = radius - 0.01;
            circle_scan.range_max = radius + 0.01;
            
            // Fill with circle data
            circle_scan.ranges.resize(num_points);
            circle_scan.intensities.resize(num_points);
            
            for (int i = 0; i < num_points; ++i) {
                // All points are exactly at radius distance
                circle_scan.ranges[i] = radius;
                circle_scan.intensities[i] = 100.0;  // Arbitrary intensity value
            }
            
            // Publish the circle
            circle_pub_->publish(circle_scan);
        }

    void scan1_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg){
        laser1_ = _msg;
        compute_scan();

    }

    void scan2_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg){
        laser2_ = _msg;

    }



    // ------------------------------MAIN FUNCTION----------------------------------------------------------------------------
    void compute_scan(){
        
        std::vector<float> scan_data;

        
        if(laser1_){
            float angle_min1, angle_incr1; // to store these vlaues from scan1 
            angle_min1 = laser1_->angle_min;
            // angle_max1 = laser1_->angle_max;
            angle_incr1 = laser1_->angle_increment;

            int count1 = laser1_->ranges.size();
            float x1, y1;
            float r1, theta1;
            float new_r1;

            for(int k = 0; k < count1; k++){
                r1 = laser1_->ranges[k]; //distance
                theta1 = (angle_min1)+ (k)*angle_incr1 - yaw_lidar1;  //angle of particular distance line

                if(std::isinf(r1)){
                    scan_data.push_back(std::numeric_limits<float>::infinity());
                }else{
                    
                    std::cout <<"hehe"<< std::endl;

                    x1 = r1 * std::cos(theta1);
                    y1 = r1 * std::sin(theta1);
                    std::cout << x1 << ", " << y1 << ", " << angle_incr1 << std::endl;
                    
                    


                    //shifting to base_link
                    x1 += x_lidar1;
                    y1 += y_lidar1;

                    std::cout << x1 << ", " << y1 << ", " << theta1 << std::endl;

                    // converting to distance and angle
                    new_r1 = sqrt(x1*x1 + y1*y1);
                    // new_theta1 = std::atan2(y1, x1);

                    //adding it scan_data
                    scan_data.push_back(new_r1);
                }

            }
        }
        if(!laser2_){
            float angle_min2, angle_incr2; // to store these vlaues from scan1 
            angle_min2 = laser2_->angle_min;
            // angle_max2 = laser2_->angle_max;
            angle_incr2 = laser2_->angle_increment;

            int count2 = laser2_->ranges.size();
            float x2, y2;
            float r2, theta2;
            float new_r2;
            for(int k = 0; k < count2; k++){
                r2 = laser2_->ranges[k]; //distance
                theta2 = angle_min2 + k*angle_incr2 - yaw_lidar2;  //angle of particular distance line

                if(std::isinf(r2)){
                    scan_data.push_back(std::numeric_limits<float>::infinity());
                }else{
                    x2 = r2 * std::cos(theta2);
                    y2 = r2 * std::sin(theta2);

                    //shifting to base_link
                    x2 += x_lidar2;
                    y2 += y_lidar2;

                    // converting to distance and angle
                    new_r2 = sqrt(x2*x2 + y2*y2);
                    // new_theta2 = std::atan2(y2, x2);

                    //adding it scan_data
                    scan_data.push_back(new_r2);
                }

            }
        }
        // pubishing scan message
        sensor_msgs::msg::LaserScan merged_scan;
        merged_scan.header.stamp = this->get_clock()->now();
        merged_scan.header.frame_id = "base_link";
        merged_scan.angle_increment = laser1_->angle_increment;
        merged_scan.angle_min = laser1_->angle_min;
        merged_scan.angle_max = laser1_->angle_max ;
        merged_scan.scan_time = laser1_->scan_time;
        merged_scan.range_max = laser1_->range_max;
        merged_scan.range_min = laser1_->range_min;
        merged_scan.scan_time = laser1_->scan_time;
        merged_scan.ranges = scan_data;

        scan_pub_->publish(merged_scan);
    }

    // -----------------------------------------------------------------------------------------------------------------------

    sensor_msgs::msg::LaserScan::SharedPtr laser1_;
    sensor_msgs::msg::LaserScan::SharedPtr laser2_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub2_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr circle_pub_;  // New publisher
    rclcpp::TimerBase::SharedPtr circle_timer_;

    // pose parameters between base_link and lidars
    float yaw_lidar1;
    float yaw_lidar2;
    float x_lidar1;  // x difference between lidar1 and base_link
    float x_lidar2;
    float y_lidar1;
    float y_lidar2;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scanMerger>());
  rclcpp::shutdown();
  return 0;
}
