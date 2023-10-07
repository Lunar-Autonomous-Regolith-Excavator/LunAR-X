#include "lx_mapping/merge_map.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

GlobalMap::GlobalMap() : Node("global_mapping_node")
{   
    // set false for dry runs, set true for printf commands and to publish occupancy grids
    debug_mode_ = false;
    double pose_x, pose_y, yaw;

    auto qos = rclcpp::SensorDataQoS();

    //subscriber
    subscription_local_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/lx_berm/occupancy_grid_3", 10, std::bind(&GlobalMap::topic_callback_local_map, this, _1)); //subscribes to the local map topic at 10Hz

    // subscription_current_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    //     "/odometry/ekf_global_node", qos, std::bind(&GlobalMap::topic_callback_current_pose, this, _1)); //subscribes to the current pose topic at 10Hz
    subscription_current_pose_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/ekf_global_node", qos, std::bind(&GlobalMap::topic_callback_current_pose, this, _1)); //subscribes to the current pose topic at 10Hz
    // rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_local_map_;

    // publishers for occupancy grids
    publisher_global_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("lx_mapping/global_map", 1);

    // configuring occupancy grid
    global_map_.header.frame_id = "map";
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

void GlobalMap::topic_callback_current_pose(const nav_msgs::msg::Odometry::SharedPtr msg){
    pose_x = msg->pose.pose.position.x;
    pose_y = msg->pose.pose.position.y;

    // from x,y,z,w to yaw
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw = yaw*180/M_PI;

    //temp
    // pose_y = 20*msg->pose.pose.position.x;
    // pose_x = 20*msg->pose.pose.position.y;

    // log pose_x and pose_y
    RCLCPP_INFO(this->get_logger(), "pose_x: %f, pose_y: %f, yaw: %f", pose_x, pose_y, yaw);
}

void GlobalMap::topic_callback_local_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Got local map");
    local_map_ = *msg;

    // transform occupancy grid using tf2
    tf2_ros::Buffer tf_buffer(this->get_clock());
    tf2_ros::TransformListener tfListener(tf_buffer);
    geometry_msgs::msg::TransformStamped transformStamped;

    try{
        transformStamped = tf_buffer.lookupTransform("base_link", "map", tf2::TimePointZero, tf2::durationFromSec(0.5));
        // display transformStamped
        // RCLCPP_INFO(this->get_logger(), "transformStamped: %f", transformStamped);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "%s",ex.what());
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        return;
    }

    // nav_msgs::msg::OccupancyGrid local_map_transformed;
    // local_map_transformed.header.frame_id = "map";
    // // apply transform to local map
    // tf2::doTransform(local_map_, local_map_transformed, transformStamped);  

    




    // displace local map by (pose_x,pose_y) and copy it to global map
    for(int i = 0; i < local_map_.info.width; i++){
        for(int j = 0; j < local_map_.info.height; j++){

            // when yaw is 0
            int global_map_idx_og = i+pose_x + (j+pose_y+global_map_.info.width)*global_map_.info.width;
            
            // compute global map index when yaw is not 0 and it is radians
            // int global_map_idx = (int)(i*cos(yaw) - j*sin(yaw) + pose_x + (i*sin(yaw) + j*cos(yaw) + pose_y + global_map_.info.width)*global_map_.info.width);
            int global_map_idx = pose_x + i*cos(yaw) - j*sin(yaw) + (pose_y + i*sin(yaw) + j*cos(yaw))*global_map_.info.width;

            RCLCPP_INFO(this->get_logger(), "global_map_idx: %d, global_map_idx_og: %d", global_map_idx, global_map_idx_og);
            // RCLCPP_INFO(this->get_logger(), "global_map_idx: %d", global_map_idx);
            // if(global_map_idx < 0 || global_map_idx > global_map_.info.width*global_map_.info.height){
            //     RCLCPP_INFO(this->get_logger(), "global_map_idx out of bounds: %d", global_map_idx);
            //     // continue;
            // }
            // else{
                int elevation  = local_map_.data[i + j*local_map_.info.width];
                if (elevation!=0 && elevation!=100){
                    // get positive mod
                    int reset_idx = (global_map_idx % (global_map_.info.width*global_map_.info.height) + (global_map_.info.width*global_map_.info.height)) % (global_map_.info.width*global_map_.info.height);
                    global_map_.data[reset_idx] = elevation;
                    // RCLCPP_INFO(this->get_logger(), "elevation: %d", elevation);
                }
            // }
        }
    }

    publisher_global_map_->publish(global_map_);
}
