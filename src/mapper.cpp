#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class FrameMapper : public rclcpp::Node {
public:
    FrameMapper() : Node("frame_mapper"){
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 
            10, 
            std::bind(&FrameMapper::odom_callback, 
            this, 
            std::placeholders::_1));

        new_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "new_odom",
            10
        );

        tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(), "Frame_mapper node just started 😃.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const {
        auto new_odom_msg = nav_msgs::msg::Odometry();
        new_odom_msg.header.stamp = msg->header.stamp;

        new_odom_msg.header.frame_id = "odom";
        new_odom_msg.child_frame_id = "base_link";
        new_odom_msg.pose.pose.position = msg->pose.pose.position;
        new_odom_msg.pose.pose.orientation = msg->pose.pose.orientation;

        new_odom_msg.pose.covariance = msg->pose.covariance;

        new_odom_msg.twist.twist.linear = msg->twist.twist.linear;
        new_odom_msg.twist.twist.angular = msg->twist.twist.angular;

        new_odom_msg.twist.covariance = msg->twist.covariance;


        new_odom_publisher_->publish(new_odom_msg);

        //create transform between odom and base_link
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = msg->header.stamp;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_link";

        //set transform data
        transform_stamped.transform.translation.x = msg->pose.pose.position.x;
        transform_stamped.transform.translation.y = msg->pose.pose.position.y;
        transform_stamped.transform.translation.z = msg->pose.pose.position.z;
        transform_stamped.transform.rotation = msg->pose.pose.orientation;

        //send transform
        tf2_broadcaster_->sendTransform(transform_stamped);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr new_odom_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameMapper>());
  rclcpp::shutdown();
  return 0;
}