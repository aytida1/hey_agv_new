#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <vector>
#include <string>

class TFPublisher : public rclcpp::Node
{
public:
    TFPublisher() : Node("tf_publisher")
    {
        // Initialize TF buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Publisher for TF data
        tf_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/all_transforms", 10);
        
        // Timer to publish transforms at 10Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&TFPublisher::publishTransforms, this)
        );
        
        // AGV IDs to track
        agv_ids_ = {"agv1", "agv2", "agv3"};
        
        RCLCPP_INFO(this->get_logger(), "TF Publisher node started - publishing AGV transforms to /all_transforms");
    }

private:
    void publishTransforms()
    {
        tf2_msgs::msg::TFMessage tf_msg;
        
        for (const auto& agv_id : agv_ids_)
        {
            // Try to get map -> base_link transform for each AGV
            std::string base_frame = agv_id + "/base_link";
            
            try
            {
                geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                    "map",
                    base_frame,
                    tf2::TimePointZero
                );
                
                // Add to the TF message
                tf_msg.transforms.push_back(transform);
                
            }
            catch (const tf2::TransformException& ex)
            {
                // Silently continue if transform not available
                continue;
            }
        }
        
        // Only publish if we have transforms
        if (!tf_msg.transforms.empty())
        {
            tf_publisher_->publish(tf_msg);
        }
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> agv_ids_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFPublisher>());
    rclcpp::shutdown();
    return 0;
}