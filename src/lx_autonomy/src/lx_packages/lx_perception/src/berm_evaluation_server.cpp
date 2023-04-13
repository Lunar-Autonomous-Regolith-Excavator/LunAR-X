#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "lx_msgs/srv/compute_berm_metrics.hpp"
#include "berm_eval/berm_map.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BermMap>();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to compute berm evaluation metrics.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}