#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <cmath>

#define GETMAXINDEX(x, y, width) ((y) * (width) + (x))

class MapPublisherNode : public rclcpp::Node {
public:
  int count;

  MapPublisherNode() : Node("map_publisher") {
    rclcpp::QoS qos(10);  // initialize to default
    qos.transient_local();
    qos.reliable();
    qos.keep_last(1);

    // Create a publisher for the map data
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos);

    // Set up a timer to periodically publish map data
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&MapPublisherNode::publishMap, this));

    count = 0;
  }

private:
  void publishMap() {
    // Populate map data (replace this with your actual map data)
    nav_msgs::msg::OccupancyGrid map_msg;
    map_msg.header.stamp = this->get_clock()->now();
    map_msg.header.frame_id = "map";

    // Set map properties
    map_msg.info.map_load_time = this->get_clock()->now();
    map_msg.info.resolution = 0.05;  // replace with your map resolution
    map_msg.info.width = 145;       // replace with your map width
    map_msg.info.height = 140;      // replace with your map height
    map_msg.info.origin.position.x = 0.1;  // replace with your map origin x
    map_msg.info.origin.position.y = 0.1;  // replace with your map origin y
    // map_msg.info.origin.position.x = -13;  // replace with your map origin x
    // map_msg.info.origin.position.y = -9;  // replace with your map origin y
    map_msg.info.origin.position.z = 0.0;
    map_msg.info.origin.orientation.x = 0.0;
    map_msg.info.origin.orientation.y = 0.0;
    map_msg.info.origin.orientation.z = 0.0;
    map_msg.info.origin.orientation.w = 1.0;

    std::vector<int8_t> map_data(map_msg.info.width * map_msg.info.height, 0);
    
    // if (count++ > 100) {
    //   RCLCPP_INFO(this->get_logger(), "Publishing map with obstacles: %d", count);
    //   // Add obstacle in x = 0 and y from [-2, 2]
    //   for (int j = 55; j < 85; j++) {
    //     for (int i = 80; i < 90; i++) {
    //       map_data[GETMAXINDEX(i, j, map_msg.info.width)] = 100;
    //     }
    //   }

    //   // Add obstacle in circles around (75, 55)
    //   for (int j = 50; j < 55; j++) {
    //     int width = std::ceil(sqrt(25 - (j - 55) * (j - 55)));
    //     for (int i = 85 - width; i < 85 + width; i++) {
    //       map_data[GETMAXINDEX(i, j, map_msg.info.width)] = 100;
    //     }
    //   }

    //   // Add obstacle in circles around (75, 85)
    //   for (int j = 85; j < 90; j++) {
    //     int width = std::ceil(sqrt(25 - (j - 85) * (j - 85)));
    //     for (int i = 85 - width; i < 85 + width; i++) {
    //       map_data[GETMAXINDEX(i, j, map_msg.info.width)] = 100;
    //     }
    //   }
    // }

    // Make top right corner of 1 meter as obstacle
    for (unsigned int j = map_msg.info.height - 1; j > map_msg.info.height - 1 - 20; j--) {
      for (unsigned int i = map_msg.info.width - 1; i > map_msg.info.width - 1 - 20; i--) {
        map_data[GETMAXINDEX(i, j, map_msg.info.width)] = 100;
      }
    }

    map_msg.data = map_data;

    // Make the borders of the map occupied
    for (unsigned int i = 0; i < map_msg.info.width; i++) {
      map_msg.data[GETMAXINDEX(i, 0, map_msg.info.width)] = 100;
      map_msg.data[GETMAXINDEX(i, map_msg.info.height - 1, map_msg.info.width)] = 100;
    }
    for (unsigned int i = 0; i < map_msg.info.height; i++) {
      map_msg.data[GETMAXINDEX(0, i, map_msg.info.width)] = 100;
      map_msg.data[GETMAXINDEX(map_msg.info.width - 1, i, map_msg.info.width)] = 100;
    }

    // Publish map data
    map_publisher_->publish(map_msg);
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
