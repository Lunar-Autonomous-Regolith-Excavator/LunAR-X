#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.h>

// For tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class RemapNode : public rclcpp::Node
{
public:
    RemapNode() : Node("remap_msgs_localization")
    {
        // Subscribe to the "odom" topic and remap it to "my_odom"
        auto qos = rclcpp::SensorDataQoS();
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/husky_velocity_controller/odom", qos,
            std::bind(&RemapNode::odom_callback, this, std::placeholders::_1));
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/husky_odom", qos);

        // pose_pub_ = create_publisher<nav_msgs::msg::Odometry>("/total_pose", qos);
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/total_station_pose_map", qos);
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/total_station_prism", qos,
        std::bind(&RemapNode::pose_callback, this, std::placeholders::_1));

        // Get transform matrix from total_station_prism to base_link
        tf2_ros::Buffer tf_buffer(this->get_clock());
        tf2_ros::TransformListener tf_listener(tf_buffer);

        // Wait for the transform between the two frames to become available
        geometry_msgs::msg::TransformStamped transform_stamped;
        try 
        {
            transform_stamped = tf_buffer.lookupTransform("base_link", "total_station_prism", tf2::TimePointZero); //primm to base link
        } 
        catch (tf2::TransformException& ex) 
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
        }

        // Convert the transform to a 4x4 matrix
        this->eigen_transform_prism_baselink = transform_stamped;
    }

private:
    geometry_msgs::msg::TransformStamped eigen_transform_prism_baselink;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Update the frame_id field in the message
        msg->header.frame_id = "base_link";
        // reset covariance
        msg->twist.covariance[0] = 0.005;
        msg->twist.covariance[7] = 0.01;
        msg->twist.covariance[14] = 0.01;
        msg->twist.covariance[21] = 0.01;
        msg->twist.covariance[28] = 0.01;
        msg->twist.covariance[35] = 0.5;
        odom_pub_->publish(*msg);
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // TODO change this implementation to use tf2 and transform accordingly
        geometry_msgs::msg::PoseWithCovarianceStamped pose_map_msg;
        
        tf2::doTransform(*msg, pose_map_msg, eigen_transform_prism_baselink);
        // pose_map_msg.pose.pose = pose_map_msg_pose;      
        // pose_map_msg.pose.covariance = msg->pose.covariance;
        pose_map_msg.header.frame_id = "map";
        pose_map_msg.header.stamp = msg->header.stamp;

        pose_pub_->publish(pose_map_msg);
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RemapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
