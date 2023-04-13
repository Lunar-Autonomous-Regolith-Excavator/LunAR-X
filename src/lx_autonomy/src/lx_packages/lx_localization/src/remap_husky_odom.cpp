#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomRemapNode : public rclcpp::Node
{
public:
  OdomRemapNode() : Node("remap_husky_odom")
  {
    // Subscribe to the "odom" topic and remap it to "my_odom"
    auto qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/husky_velocity_controller/odom", qos,
      std::bind(&OdomRemapNode::odom_callback, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/husky_odom", qos);
  }

private:
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

    std::cout<<"odom msgs"<<msg->twist.twist.linear.x<<" "<<msg->twist.twist.angular.z<<std::endl;;
    // msg->child_frame_id = "";
    // Publish the modified message on the "my_odom" topic
    odom_pub_->publish(*msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomRemapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

//steps to add it to CMakeLists.txt
//1. add_executable(remap_husky_odom src/remap_husky_odom.cpp)
//2. ament_target_dependencies(remap_husky_odom rclcpp nav_msgs)
//3. install(TARGETS remap_husky_odom DESTINATION lib/${PROJECT_NAME})
