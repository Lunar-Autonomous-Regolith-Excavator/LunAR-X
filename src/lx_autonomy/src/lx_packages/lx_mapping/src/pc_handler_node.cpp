#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "lx_mapping/pc_handler.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudHandler>();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to process the pointclouds.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}