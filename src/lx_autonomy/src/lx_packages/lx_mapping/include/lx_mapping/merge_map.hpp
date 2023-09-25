#include <cstdio>
#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include "lx_msgs/srv/berm_metrics.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class GlobalMap : public rclcpp::Node
{
public:
    GlobalMap();

private:

    void topic_callback_local_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void topic_callback_current_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_local_map_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_current_pose_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_global_map_;
    
    double pose_x, pose_y;
    bool debug_mode_;
    // auto qos;

    bool is_initialized_right_, is_initialized_left_;

    nav_msgs::msg::OccupancyGrid local_map_, global_map_;
    geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;
    rclcpp::TimerBase::SharedPtr timer_;
};  