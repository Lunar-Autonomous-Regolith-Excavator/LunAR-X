#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class GoalConverter : public rclcpp::Node
{
public:
  GoalConverter() : Node("goal_converter")
  {
    // Subscribe to /goal_pose topic
    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&GoalConverter::goalPoseCallback, this, std::placeholders::_1));

    // Publish to /husky_goal topic
    pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/husky_goal", 10);
  }

private:
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Publish the received message on /husky_goal topic
    pub_->publish(*msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalConverter>());
  rclcpp::shutdown();
  return 0;
}
