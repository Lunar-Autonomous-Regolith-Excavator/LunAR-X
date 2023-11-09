#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lx_msgs/action/auto_nav.hpp"

namespace rclcpp = rclcpp;
namespace rclcpp_action = rclcpp_action;

using AutoNav = lx_msgs::action::AutoNav;
using GoalHandleAutoNav = rclcpp_action::ClientGoalHandle<AutoNav>;

class AutoNavClient : public rclcpp::Node
{
public:
  explicit AutoNavClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("auto_nav_client", options)
  {
    client_ = rclcpp_action::create_client<AutoNav>(this, "/operations/autonav_action");
    
    // Wait for the action server to be available
    if (!client_->wait_for_action_server(std::chrono::seconds(10)))
    {
      RCLCPP_ERROR(get_logger(), "Action server not available");
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Action server available, sending goal...");
      sendGoal();
    }
  }

private:
  void sendGoal()
  {
    auto goal_msg = AutoNav::Goal();

    // Fill in the goal_msg with your desired pose information
    goal_msg.goal.header.frame_id = "map";
    goal_msg.goal.header.stamp = now();

    //     x: 5.915783645379575
    //   y: 1.87090889990329
    //   z: 0.33134786609766387
    // orientation:
    //   x: 0.007638247913776921
    //   y: -0.012599615553454575
    //   z: -0.4658512491429855
    //   w: 0.8847403689945614

    // set goal as the above pose
    // goal_msg.goal.pose.position.x = 5.915783645379575;
    // goal_msg.goal.pose.position.y = 1.87090889990329;
    // goal_msg.goal.pose.position.z = 0.33134786609766387;
    // goal_msg.goal.pose.orientation.x = 0.007638247913776921;
    // goal_msg.goal.pose.orientation.y = -0.012599615553454575;
    // goal_msg.goal.pose.orientation.z = -0.4658512491429855;
    // goal_msg.goal.pose.orientation.w = 0.8847403689945614;
    goal_msg.goal.pose.position.x = 5;
    goal_msg.goal.pose.position.y = 2;



    auto send_goal_options = rclcpp_action::Client<AutoNav>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&AutoNavClient::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.result_callback =
      std::bind(&AutoNavClient::resultCallback, this, std::placeholders::_1);

    client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goalResponseCallback(const GoalHandleAutoNav::SharedPtr &goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void resultCallback(const GoalHandleAutoNav::WrappedResult &result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Goal succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(get_logger(), "Goal was aborted");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        break;
    }
  }

  rclcpp_action::Client<AutoNav>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoNavClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
