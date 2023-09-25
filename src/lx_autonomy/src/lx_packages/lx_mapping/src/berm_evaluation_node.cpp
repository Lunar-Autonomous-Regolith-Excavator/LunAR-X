#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "lx_mapping/berm_evaluation.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BermMap>();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to compute berm evaluation metrics.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}