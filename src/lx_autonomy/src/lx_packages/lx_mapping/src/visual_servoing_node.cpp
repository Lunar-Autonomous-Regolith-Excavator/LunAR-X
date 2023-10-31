#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "lx_mapping/visual_servoing.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisualServoing>();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to output visual servo.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}