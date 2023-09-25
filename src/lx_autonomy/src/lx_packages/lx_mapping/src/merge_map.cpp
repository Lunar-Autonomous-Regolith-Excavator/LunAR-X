#include "lx_mapping/merge_map.hpp"

GlobalMap::GlobalMap() : Node("global_mapping_node")
{   
    // set false for dry runs, set true for printf commands and to publish occupancy grids
    debug_mode_ = false;
    double pose_x, pose_y;

    auto qos = rclcpp::SensorDataQoS();

    //subscriber
    subscription_local_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/lx_berm/occupancy_grid_3", 10, std::bind(&GlobalMap::topic_callback_local_map, this, _1)); //subscribes to the local map topic at 10Hz

    subscription_current_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/total_station_pose_map", qos, std::bind(&GlobalMap::topic_callback_current_pose, this, _1)); //subscribes to the current pose topic at 10Hz
    // rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_local_map_;

    // publishers for occupancy grids
    publisher_global_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("lx_mapping/global_map", 10);

    // configuring occupancy grid
    global_map_.header.frame_id = "base_link";
    global_map_.info.resolution = 0.1;
    global_map_.info.width = 150;
    global_map_.info.height = 150;
    global_map_.info.origin.position.x = -5;
    global_map_.info.origin.position.y = -5;
    global_map_.info.origin.position.z = 0;
    global_map_.info.origin.orientation.x = 0;
    global_map_.info.origin.orientation.y = 0;
    global_map_.info.origin.orientation.z = 0;
    global_map_.info.origin.orientation.w = 1;

    // initializing occupancy grid
    global_map_.data.resize(global_map_.info.width*global_map_.info.height);
    for(int i = 0; i < global_map_.info.width*global_map_.info.height; i++){
        global_map_.data[i] = 0;
    }
}

void GlobalMap::topic_callback_current_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
    // pose_x = msg->pose.pose.position.x*20;
    // pose_y = msg->pose.pose.position.y*20;
    //temp
    pose_y = 20*msg->pose.pose.position.x;
    pose_x = 20*msg->pose.pose.position.y;

    // log pose_x and pose_y
    RCLCPP_INFO(this->get_logger(), "pose_x: %f, pose_y: %f", pose_x, pose_y);
}

void GlobalMap::topic_callback_local_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Got local map");
    local_map_ = *msg;

    // displace local map by (pose_x,pose_y) and copy it to global map
    for(int i = 0; i < local_map_.info.width; i++){
        for(int j = 0; j < local_map_.info.height; j++){
            int global_map_idx = i+pose_x + (j+pose_y+global_map_.info.width/2)*global_map_.info.width;
            // RCLCPP_INFO(this->get_logger(), "global_map_idx: %d", global_map_idx);
            if(global_map_idx < 0 || global_map_idx > global_map_.info.width*global_map_.info.height){
                RCLCPP_INFO(this->get_logger(), "global_map_idx out of bounds: %d", global_map_idx);
                // continue;
            }
            // else{
                int elevation  = local_map_.data[i + j*local_map_.info.width];
                if (elevation!=0 && elevation!=100){
                    global_map_.data[global_map_idx % (global_map_.info.width*global_map_.info.height)] = elevation;
                    RCLCPP_INFO(this->get_logger(), "elevation: %d", elevation);
                }
            // }
        }
    }

    publisher_global_map_->publish(global_map_);
}
