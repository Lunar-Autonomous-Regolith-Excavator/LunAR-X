#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "lx_mapping/merge_map.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalMap>();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to integrate local map into global map.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}