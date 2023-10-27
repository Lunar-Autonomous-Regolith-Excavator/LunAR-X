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
#include <tf2_ros/buffer.h>

#include "lx_msgs/srv/map.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose_array.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class WorldModel : public rclcpp::Node
{
public:
    WorldModel();

private:
    rclcpp::Service<lx_msgs::srv::Map>::SharedPtr service_map_;

    void startStopMappingCallback(const std::shared_ptr<lx_msgs::srv::Map::Request> request,
        std::shared_ptr<lx_msgs::srv::Map::Response> response);

    void topic_callback_fuse_map(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void topic_callback_global_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void topic_callback_zones(const nav_msgs::msg::Odometry::SharedPtr msg);

    void topic_callback_current_pose(const nav_msgs::msg::Odometry::SharedPtr msg);

    void topic_callback_aruco_poses(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    void transform_pc_cam2map(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pc_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_global_map_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pc_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_global_map_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_world_model_;
    
    bool debug_mode_;
    sensor_msgs::msg::PointCloud2 pointcloud;
    bool got_pointcloud = false;
    // auto qos;

    nav_msgs::msg::OccupancyGrid global_map_, elevation_costmap_, slope_costmap_, berm_costmap_, zone_costmap_, world_model_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock clock;

    double tool_height_wrt_base_link_;
    double pose_x, pose_y, pose_z;
    double robot_roll, robot_pitch, robot_yaw;

    double min_x, min_y, max_x, max_y;
    int min_col, min_row, max_col, max_row;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};  