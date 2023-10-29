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
#include "lx_msgs/srv/map.hpp"

// header file for nav_msgs::msg::Odometry
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose_array.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

class VisualServoing : public rclcpp::Node
{
public:
    VisualServoing();

private:

    rclcpp::Service<lx_msgs::srv::Map>::SharedPtr service_vs_;

    void topic_callback_current_pose(const nav_msgs::msg::Odometry::SharedPtr msg);

    void topic_callback_vs(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void topic_callback_aruco_poses(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    void startStopVSCallback(const std::shared_ptr<lx_msgs::srv::Map::Request> request,
        std::shared_ptr<lx_msgs::srv::Map::Response> response);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pc_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_aruco_poses_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_vs_error_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pc_;

    double pose_x, pose_y, pose_z;
    double robot_roll, robot_pitch, robot_yaw;
    bool debug_mode_;
    double tool_height_wrt_base_link_;
    // auto qos;

    bool is_initialized_right_, is_initialized_left_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Clock clock;
};  
