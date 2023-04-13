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
#include <tf2_eigen/tf2_eigen.h>


using std::placeholders::_1;
using namespace std::chrono_literals;
using ComputeBermMetrics = lx_msgs::srv::ComputeBermMetrics;

class BermMap : public rclcpp::Node
{
public:
    BermMap();

    void setTargetBermHeight(double height = 0.15)
    {
        berm_target_height = height;
    }

private:

    void service_callback(const std::shared_ptr<ComputeBermMetrics::Request> request,
    std::shared_ptr<ComputeBermMetrics::Response> response)
    {
        process_left(msg_left);
        process_right(msg_right);
        response->height = berm_height;
        response->width = berm_width;
        response->length = berm_length;
        RCLCPP_INFO(this->get_logger(), "Computed berm metrics: height: %f, width: %f, length: %f", response->height, response->width, response->length);
    }

    void add_dune_neighbors(std::vector<int> &dune_x, std::vector<int> &dune_y, std::vector<int> &dune_indices, int idx, int width);

    void grow_dune(std::vector<int> &dune_indices,int &score, int idx, int width, int dune_counter);

    void topic_callback_right(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        msg_right = msg;
    }
    void topic_callback_left(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        msg_left = msg;
    }

    void process_right(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void process_left(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_right, subscription_left;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_og, publisher_fil, publisher_pc_density_right, publisher_pc_density_left;
    rclcpp::Service<ComputeBermMetrics>::SharedPtr service_;

    double berm_target_height;
    float berm_height, berm_width, berm_length;
    bool debug_mode;

    nav_msgs::msg::OccupancyGrid occupancy_grid, filtered_occupancy_grid;
    nav_msgs::msg::OccupancyGrid pc_density_grid_right, pc_density_grid_left;
    std_msgs::msg::Float32 berm_height_msg;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::PointCloud2::SharedPtr msg_right, msg_left;
};