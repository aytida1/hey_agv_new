#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>

class TagTransformNode : public rclcpp::Node
{
public:
    TagTransformNode() : Node("tag_transform_node")
    {
        // Initialize TF2 buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Create publisher for detected dock pose
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/detected_dock_pose", 10);
        
        // Create timer to periodically publish transform
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TagTransformNode::publish_transform, this));
        
        // Parameters for frame names
        this->declare_parameter("base_frame", "map");
        this->declare_parameter("tag_frame", "tag36h11:0");
        this->declare_parameter("publish_rate", 10.0);
        
        base_frame_ = this->get_parameter("base_frame").as_string();
        tag_frame_ = this->get_parameter("tag_frame").as_string();
        
        double rate = this->get_parameter("publish_rate").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&TagTransformNode::publish_transform, this));
        
        RCLCPP_INFO(this->get_logger(), "Tag Transform Node started");
        RCLCPP_INFO(this->get_logger(), "Publishing transform from %s to %s on /detected_dock_pose", 
                   base_frame_.c_str(), tag_frame_.c_str());
    }

private:
    void publish_transform()
    {
        try
        {
            // Get transform from base_link to tag
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(
                base_frame_, tag_frame_, tf2::TimePointZero);
            
            // Convert transform to pose stamped
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = this->now();
            pose_stamped.header.frame_id = base_frame_;
            
            // Set position from transform translation
            pose_stamped.pose.position.x = transform_stamped.transform.translation.x;
            pose_stamped.pose.position.y = transform_stamped.transform.translation.y;
            pose_stamped.pose.position.z = transform_stamped.transform.translation.z;
            
            // Set orientation from transform rotation
            pose_stamped.pose.orientation.x = transform_stamped.transform.rotation.x;
            pose_stamped.pose.orientation.y = transform_stamped.transform.rotation.y;
            pose_stamped.pose.orientation.z = transform_stamped.transform.rotation.z;
            pose_stamped.pose.orientation.w = transform_stamped.transform.rotation.w;
            
            // Publish the pose
            pose_publisher_->publish(pose_stamped);

           
            
            RCLCPP_INFO(this->get_logger(), 
                        "Published pose: x=%.3f, y=%.3f, z=%.3f", 
                        pose_stamped.pose.position.x,
                        pose_stamped.pose.position.y, 
                        pose_stamped.pose.position.z);
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_WARN(this->get_logger(), 
                       "Could not transform %s to %s: %s", 
                       tag_frame_.c_str(), base_frame_.c_str(), ex.what());
        }
    }
    
    // TF2 components
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Publisher and timer
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Frame names
    std::string base_frame_;
    std::string tag_frame_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TagTransformNode>());
    rclcpp::shutdown();
    return 0;
}