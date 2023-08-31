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
using BermMetrics = lx_msgs::srv::BermMetrics;

class BermMap : public rclcpp::Node
{
public:
    BermMap();

    void setTargetBermHeight(double height = 0.15);

private:

    void service_callback(const std::shared_ptr<BermMetrics::Request> request,
    std::shared_ptr<BermMetrics::Response> response);

    void add_dune_neighbors(std::vector<int> &dune_x, std::vector<int> &dune_y, std::vector<int> &dune_indices, int idx, int width);

    void grow_dune(std::vector<int> &dune_indices,int &score, int idx, int width, int dune_counter, int rec_count);

    void topic_callback_right(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    bool process_right(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_right_, subscription_left_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_og_, publisher_fil_, publisher_pc_density_right_, publisher_pc_density_left_;
    rclcpp::Service<BermMetrics>::SharedPtr service_;

    double berm_target_height_;
    float berm_height_, berm_width_, berm_length_;
    bool debug_mode_;

    bool is_initialized_right_, is_initialized_left_;

    nav_msgs::msg::OccupancyGrid occupancy_grid_, filtered_occupancy_grid_;
    nav_msgs::msg::OccupancyGrid pc_density_grid_right_, pc_density_grid_left_;
    std_msgs::msg::Float32 berm_height_msg_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::PointCloud2::SharedPtr msg_right_, msg_left_;
};  