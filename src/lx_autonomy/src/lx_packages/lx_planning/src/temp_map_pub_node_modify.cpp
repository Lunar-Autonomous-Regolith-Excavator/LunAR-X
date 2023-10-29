#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class MapModifierNode : public rclcpp::Node {
public:
  MapModifierNode() : Node("map_modifier") {

    rclcpp::QoS qos(10);  // initialize to default
    qos.transient_local();
    qos.reliable();
    qos.keep_last(1);

    // Create a subscriber for the map data
    map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", qos, std::bind(&MapModifierNode::mapCallback, this, std::placeholders::_1));

    // Create a publisher for the modified map data
    modified_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("modified_map", qos);
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg) {
    // Modify all free cells by randomly sampling
    nav_msgs::msg::OccupancyGrid modified_map_msg = *map_msg;

    for (size_t i = 0; i < modified_map_msg.data.size(); ++i) {
      if (modified_map_msg.data[i] < 50) {  // Free cell
        modified_map_msg.data[i] = rand() % 50;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Modified map data");z

    // Publish the modified map data
    modified_map_publisher_->publish(modified_map_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr modified_map_publisher_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapModifierNode>());
  rclcpp::shutdown();
  return 0;
}
