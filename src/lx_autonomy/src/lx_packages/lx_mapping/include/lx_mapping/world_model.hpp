#include <cstdio>
#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "lx_msgs/srv/berm_metrics.hpp"

// header file for nav_msgs::msg::Odometry
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


using std::placeholders::_1;
using namespace std::chrono_literals;

class WorldModel : public rclcpp::Node
{
public:
    WorldModel();

private:

    void topic_callback_global_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void topic_callback_zones(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_global_map_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_world_model_;
    
    bool debug_mode_;
    double tool_height_wrt_base_link_;
    // auto qos;

    nav_msgs::msg::OccupancyGrid global_map_, elevation_costmap_, slope_costmap_, berm_costmap_, zone_costmap_, world_model_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock clock;
};  