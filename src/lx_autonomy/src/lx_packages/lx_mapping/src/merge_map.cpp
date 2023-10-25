#include "lx_mapping/merge_map.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>

// tf2_geometry_msgs
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// pcl cropbox
#include <pcl/filters/crop_box.h>
// pcl plane segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
// pcl transform
#include <pcl/common/transforms.h>
// pcl noise removal
#include <pcl/filters/statistical_outlier_removal.h>
// pcl passthrough
#include <pcl/filters/passthrough.h>
#include<vector>
#include<iostream>


#include <memory>

GlobalMap::GlobalMap() : Node("global_mapping_node")
{   
    // set false for dry runs, set true for printf commands and to publish occupancy grids
    debug_mode_ = true;
    double pose_x, pose_y, pose_z, yaw;
    int scale = 10;

    auto qos = rclcpp::SensorDataQoS();

    // not defined
    tool_height_wrt_base_link_ = 1000;
    min_x=1000.0; min_y=1000.0; max_x=-1000.0; max_y=-1000.0;

    double robot_roll = 0;
    double robot_pitch = 0;
    double robot_yaw = 0;

    subscription_current_pose_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/ekf_global_node", qos, std::bind(&GlobalMap::topic_callback_current_pose, this, _1)); //subscribes to the current pose topic at 10Hz    // publishers for occupancy grids
    
    publisher_global_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("lx_mapping/global_map", 10);

    subscription_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "camera/depth/color/points", 1, std::bind(&GlobalMap::topic_callback_pc, this, _1)); //subscribes to the point cloud topic at 1Hz

    publisher_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lx_mapping/transformed_pc", 10);

    subscription_aruco_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/aruco_poses", 1, std::bind(&GlobalMap::topic_callback_aruco_poses, this, _1)); //subscribes to the aruco markers topic at 10Hz

    publisher_tool_height_ = this->create_publisher<std_msgs::msg::Float32>("lx_mapping/tool_height", 10);

    // configuring occupancy grid
    global_map_.header.frame_id = "map";
    global_map_.info.resolution = 0.05;
    global_map_.info.width = 160;
    global_map_.info.height = 160;
    global_map_.info.origin.position.x = -4;
    global_map_.info.origin.position.y = -4;
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

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(100000000));    
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    double max_x = -1000.0;
    double max_y = -1000.0;
    double min_x = 1000.0;
    double min_y = 1000.0;
    int min_col = 1000, min_row = 1000, max_col = -1000, max_row = -1000;

}

void GlobalMap::topic_callback_aruco_poses(const geometry_msgs::msg::PoseArray::SharedPtr msg){
   if(msg->poses.size()>0){
        geometry_msgs::msg::Pose aruco_pose = msg->poses[0];
        tool_height_wrt_base_link_ = -(aruco_pose.position.y* 0.7349 + aruco_pose.position.z* 0.2389 -0.2688)/0.6346;
   }
}

void GlobalMap::topic_callback_pc(const sensor_msgs::msg::PointCloud2::SharedPtr msg)  {

    // Lookup the transform from the sensor frame to the target frame
    geometry_msgs::msg::TransformStamped cam2map_transform;
    try
    {
      cam2map_transform = tf_buffer_->lookupTransform("camera_link", "map",tf2::TimePointZero, tf2::durationFromSec(1));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", ex.what());
      RCLCPP_WARN(this->get_logger(), "C");
      return;
    }
  
    // convert x,y,z,w to roll, pitch, yaw
    tf2::Quaternion q(cam2map_transform.transform.rotation.x, cam2map_transform.transform.rotation.y, cam2map_transform.transform.rotation.z, cam2map_transform.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // RCLCPP_INFO(this->get_logger(), "roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
    // yaw += 1.57;


    // crop the point cloud
    sensor_msgs::msg::PointCloud2 cropped_msg;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // The same rotation matrix as before; theta radians around Z axis

    //remove noise from the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);


    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    // Define a translation of 2.5 meters on the x axis.
    // transform_2.translation() << 0, 0, 0.8;

    transform_2.rotate(Eigen::AngleAxisf(-roll+3*3.1415/180, Eigen::Vector3f::UnitZ()));
    transform_2.rotate(Eigen::AngleAxisf(pitch+2*3.1415/180, Eigen::Vector3f::UnitX()));
    transform_2.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitY()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, transform_2);

    // RCLCPP_INFO(this->get_logger(), "tool_height_wrt_base_link_: %f", tool_height_wrt_base_link_);


    // MODE: TRAVERSAL
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(transformed_cloud);
    if(tool_height_wrt_base_link_>0.2)
        crop_box.setMin(Eigen::Vector4f(-10, 0.8-tool_height_wrt_base_link_, -10, 1.0));
    else
        crop_box.setMin(Eigen::Vector4f(-10, 0.5, -10, 1.0));
    crop_box.setMax(Eigen::Vector4f(10, 1 ,10, 1.0));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_local_map(new pcl::PointCloud<pcl::PointXYZ>);
    crop_box.filter(*cropped_cloud_local_map);
    
    if(cropped_cloud_local_map->points.size() < 100){
        RCLCPP_INFO(this->get_logger(), "cropped_cloud is empty");
        return;
    }

    // create an elevation map from the inliers of the type occupancygrid
    nav_msgs::msg::OccupancyGrid elevation_map;
    nav_msgs::msg::OccupancyGrid local_elevation_map;
    nav_msgs::msg::OccupancyGrid pd_map;
    elevation_map.header.frame_id = "map";
    elevation_map.info=global_map_.info;

    // populate values as 0 in the local elevation map and pd map
    local_elevation_map.header.frame_id = "map";
    local_elevation_map.info=global_map_.info;
    pd_map.header.frame_id = "map";
    pd_map.info=global_map_.info;
    local_elevation_map.data.resize(local_elevation_map.info.width*local_elevation_map.info.height);
    pd_map.data.resize(pd_map.info.width*pd_map.info.height);

    // RCLCPP_INFO(this->get_logger(), "local_elevation_map size: %d", local_elevation_map.data.size());

    for(int i = 0; i < local_elevation_map.info.width*local_elevation_map.info.height; i++){
        local_elevation_map.data[i] = 0;
        pd_map.data[i] = 0;
    }
    // calculate average y-values of inliers in the square patch with corners (x=0.05,z=0.45) and (x=0.1, z = 0.5)
    double sum_y = 0;
    int count = 0;

    // array with double to store elevation vlaues
    double elevation_values[global_map_.info.width*global_map_.info.height];
    double density_values[global_map_.info.width*global_map_.info.height];
    for(int i = 0; i < global_map_.info.width*global_map_.info.height; i++){
        elevation_values[i] = 0.0;
        density_values[i] = 0.0;
    }
    
    for(int i = 0; i < cropped_cloud_local_map->points.size(); i++){
        float x_part = (cropped_cloud_local_map->points[i].x*cos(robot_yaw) - (cropped_cloud_local_map->points[i].z)*sin(robot_yaw));
        float y_part = (cropped_cloud_local_map->points[i].x*sin(robot_yaw) + (cropped_cloud_local_map->points[i].z)*cos(robot_yaw));
        int col_x = int((pose_x+x_part)/global_map_.info.resolution);
        int row_y = int((pose_y+y_part)/global_map_.info.resolution);
        if(col_x < min_col){
            min_col = col_x;
        }
        if(row_y < min_row){
            min_row = row_y;
        }
        if(col_x > max_col){
            max_col = col_x;
        }
        if(row_y > max_row){
            max_row = row_y;
        }
        col_x = col_x % 160;
        row_y = row_y % 160;
        if(col_x < 0){
            col_x += 160;
        }
        if(row_y < 0){
            row_y += 160;
        }
        int global_idx = col_x + row_y*global_map_.info.width;
        double elev = (cropped_cloud_local_map->points[i].y) + pose_z;
        elevation_values[global_idx] += 100.0*(elev-2.0);
        density_values[global_idx] += 1.0;
    }

    // where pd_map is not 0, divide local_elevation_map by pd_map
    if(robot_yaw!=0){
        for(int i = 0; i < global_map_.info.width*global_map_.info.height; i++){
            if(density_values[i] > 1.0){
                // RCLCPP_INFO(this->get_logger(), "i :%d, density_values[i]: %f, elevation_values[i]: %f", i,density_values[i], elevation_values[i]);
                global_map_.data[i] = int(elevation_values[i]/density_values[i]);
                // elevation_values[i] = 0.0;    
                // density_values[i] = 0.0;
                // RCLCPP_INFO(this->get_logger(), "global_map_.data[i]: %d", global_map_.data[i]);
            }
        }
    }

    // Convert the cropped point cloud back to a PointCloud2 message
    sensor_msgs::msg::PointCloud2 result_msg;
    // RCLCPP_INFO(this->get_logger(), "G, %d", cropped_cloud_local_map->points.size());
    pcl::toROSMsg(*cloud_filtered, result_msg);
    // Publish the transformed message
    publisher_pc_->publish(result_msg);
    publisher_global_map_->publish(global_map_);
  }





void GlobalMap::topic_callback_current_pose(const nav_msgs::msg::Odometry::SharedPtr msg){
    pose_x = msg->pose.pose.position.x + 4.0;
    pose_y = msg->pose.pose.position.y + 0.0;
    pose_z = msg->pose.pose.position.z; 

    if(pose_x < min_x){
        min_x = pose_x;
    }
    if(pose_y < min_y){
        min_y = pose_y;
    }
    if(pose_x > max_x){
        max_x = pose_x;
    }
    if(pose_y > max_y){
        max_y = pose_y;
    }

    // RCLCPP_INFO(this->get_logger(), "pose_x: %f, pose_y: %f, pose_z: %f", pose_x, pose_y, pose_z);
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(robot_roll, robot_pitch, robot_yaw);
    robot_yaw += 1.5708;
    // robot_yaw += 3.1415;
}