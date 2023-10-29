#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"

namespace rclcpp = rclcpp;
namespace rclcpp_action = rclcpp_action;

using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using GoalHandleComputePathToPose = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

class ComputePathToPoseClient : public rclcpp::Node
{
public:
  explicit ComputePathToPoseClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("compute_path_to_pose_client", options)
  {
    client_ = rclcpp_action::create_client<ComputePathToPose>(this, "compute_path_to_pose");
    
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
    auto goal_msg = ComputePathToPose::Goal();

    // Fill in the goal_msg with your desired pose information
    goal_msg.goal.header.frame_id = "map";
    goal_msg.goal.header.stamp = now();
    goal_msg.goal.pose.position.x = 0.0;
    goal_msg.goal.pose.position.y = 0.0;

    goal_msg.planner_id = "GridBased";

    goal_msg.use_start = false;

    auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ComputePathToPoseClient::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.result_callback =
      std::bind(&ComputePathToPoseClient::resultCallback, this, std::placeholders::_1);

    client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goalResponseCallback(const GoalHandleComputePathToPose::SharedPtr &goal_handle)
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

  void resultCallback(const GoalHandleComputePathToPose::WrappedResult &result)
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

  rclcpp_action::Client<ComputePathToPose>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ComputePathToPoseClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
