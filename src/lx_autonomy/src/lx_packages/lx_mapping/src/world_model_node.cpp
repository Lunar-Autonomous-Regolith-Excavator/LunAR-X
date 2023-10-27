#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "lx_mapping/world_model.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WorldModel>();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to make the world model.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}