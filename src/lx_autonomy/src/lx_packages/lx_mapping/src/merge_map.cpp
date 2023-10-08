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


#include <memory>

GlobalMap::GlobalMap() : Node("global_mapping_node")
{   
    // set false for dry runs, set true for printf commands and to publish occupancy grids
    debug_mode_ = false;
    double pose_x, pose_y, pose_z, yaw;
    int scale = 10;

    auto qos = rclcpp::SensorDataQoS();

    // not defined
    tool_height_wrt_base_link_ = -1000;
    min_x=1000.0; min_y=1000.0; max_x=-1000.0; max_y=-1000.0;

    double robot_roll = 0;
    double robot_pitch = 0;
    double robot_yaw = 0;

    // subscription_local_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    //     "/lx_berm/occupancy_grid_3", 10, std::bind(&GlobalMap::topic_callback_local_map, this, _1)); //subscribes to the local map topic at 10Hz

    subscription_current_pose_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/ekf_global_node", qos, std::bind(&GlobalMap::topic_callback_current_pose, this, _1)); //subscribes to the current pose topic at 10Hz    // publishers for occupancy grids
    
    publisher_global_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("lx_mapping/global_map", 1);

    subscription_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "camera/depth/color/points", 10, std::bind(&GlobalMap::topic_callback_pc, this, _1)); //subscribes to the point cloud topic at 1Hz

    publisher_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lx_mapping/transformed_pc", 10);

    subscription_aruco_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/aruco_poses", 10, std::bind(&GlobalMap::topic_callback_aruco_poses, this, _1)); //subscribes to the aruco markers topic at 10Hz

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

    // buffer of 2 elements
    // make it same type as the clock but value as 1696459789.087897 tf_buffer_ = std::make_shared<tf2_ros::Buffer>(1696459789.087897, tf2::durationFromSec(30.0));
    // tf_buffer_ = std::make_shared<tf2_ros::Buffer>( rclcpp::Clock (1696459789.087897), tf2::durationFromSec(30.0));
    // create a clock with the same type as the clock but value as 1696459789.087897
    // rclcpp::Time custom_time(1696459789);
    // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(custom_time, tf2::durationFromSec(30.0));
    
    // create a clock with the same type as the clock but value as 1696459789.087897
    // type of this->get_clock() is rclcpp::Clock
    // rclcpp::Time custom_time(1696459789);
    // rclcpp::Clock custom_clock(custom_time);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(10000000));    
    
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}

void GlobalMap::topic_callback_aruco_poses(const geometry_msgs::msg::PoseArray::SharedPtr msg){
   if(msg->poses.size()>0){
        geometry_msgs::msg::Pose aruco_pose = msg->poses[0];
        tool_height_wrt_base_link_ = -(aruco_pose.position.y* 0.7349 + aruco_pose.position.z* 0.2389 -0.2688)/0.6346;
   }
}

void GlobalMap::topic_callback_pc(const sensor_msgs::msg::PointCloud2::SharedPtr msg)  {
    // Lookup the transform from the sensor frame to the target frame
    geometry_msgs::msg::TransformStamped transform;
    // RCLCPP_INFO(this->get_logger(), "Got point cloud");
    try
    {
      transform = tf_buffer_->lookupTransform("camera_link", "base_link",tf2::TimePointZero, tf2::durationFromSec(1));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", ex.what());
      return;
    }
    

    // Transform the PointCloud2 message
    // sensor_msgs::msg::PointCloud2 transformed_msg;
    // tf2::doTransform(*msg, transformed_msg, transform);

    // // print some info from the transform
    // RCLCPP_INFO(this->get_logger(), "Translation: %f, %f, %f", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
    // RCLCPP_INFO(this->get_logger(), "Rotation: %f, %f, %f, %f", transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
    // // print the frame_id of the transformed message
    // RCLCPP_INFO(this->get_logger(), "Transformed message frame_id: %s", transformed_msg.header.frame_id.c_str());

    // convert x,y,z,w to roll, pitch, yaw
    tf2::Quaternion q(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // RCLCPP_INFO(this->get_logger(), "roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);

    // crop the point cloud
    sensor_msgs::msg::PointCloud2 cropped_msg;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // calculate mean and standard deviation of x,y,z values of the point cloud
    double mean_x = 0, mean_y = 0, mean_z = 0, std_x = 0, std_y = 0, std_z = 0;
    for(int i = 0; i < cloud->points.size(); i++){
        mean_x += cloud->points[i].x;
        mean_y += cloud->points[i].y;
        mean_z += cloud->points[i].z;
    }
    mean_x /= cloud->points.size();
    mean_y /= cloud->points.size();
    mean_z /= cloud->points.size();
    for(int i = 0; i < cloud->points.size(); i++){
        std_x += (cloud->points[i].x-mean_x)*(cloud->points[i].x-mean_x);
        std_y += (cloud->points[i].y-mean_y)*(cloud->points[i].y-mean_y);
        std_z += (cloud->points[i].z-mean_z)*(cloud->points[i].z-mean_z);
    }
    std_x /= cloud->points.size();
    std_y /= cloud->points.size();
    std_z /= cloud->points.size();
    std_x = sqrt(std_x);
    std_y = sqrt(std_y);
    std_z = sqrt(std_z);
    // print
    // RCLCPP_INFO(this->get_logger(), "Before transformation: mean_x: %f, mean_y: %f, mean_z: %f, std_x: %f, std_y: %f, std_z: %f", mean_x, mean_y, mean_z, std_x, std_y, std_z);


    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    // Define a translation of 2.5 meters on the x axis.
    transform_2.translation() << 0, 0.0, 0.0;

    // The same rotation matrix as before; theta radians around Z axis
    transform_2.rotate(Eigen::AngleAxisf(-roll+3*3.1415/180, Eigen::Vector3f::UnitZ()));
    transform_2.rotate(Eigen::AngleAxisf(pitch+2*3.1415/180, Eigen::Vector3f::UnitX()));
    transform_2.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitY()));

    //remove noise from the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, transform_2);

    // calculate mean and standard deviation of x,y,z values of the point cloud
    mean_x = 0; mean_y = 0; mean_z = 0; std_x = 0; std_y = 0; std_z = 0;
    for(int i = 0; i < transformed_cloud->points.size(); i++){
        mean_x += transformed_cloud->points[i].x;
        mean_y += transformed_cloud->points[i].y;
        mean_z += transformed_cloud->points[i].z;
    }
    mean_x /= transformed_cloud->points.size();
    mean_y /= transformed_cloud->points.size();
    mean_z /= transformed_cloud->points.size();
    for(int i = 0; i < transformed_cloud->points.size(); i++){
        std_x += (transformed_cloud->points[i].x-mean_x)*(transformed_cloud->points[i].x-mean_x);
        std_y += (transformed_cloud->points[i].y-mean_y)*(transformed_cloud->points[i].y-mean_y);
        std_z += (transformed_cloud->points[i].z-mean_z)*(transformed_cloud->points[i].z-mean_z);
    }
    std_x /= transformed_cloud->points.size();
    std_y /= transformed_cloud->points.size();
    std_z /= transformed_cloud->points.size();
    std_x = sqrt(std_x);
    std_y = sqrt(std_y);
    std_z = sqrt(std_z);
    // print
    // RCLCPP_INFO(this->get_logger(), "After transformation: mean_x: %f, mean_y: %f, mean_z: %f, std_x: %f, std_y: %f, std_z: %f", mean_x, mean_y, mean_z, std_x, std_y, std_z);

    // Create a CropBox filter and set the region of interest
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(transformed_cloud);
    crop_box.setMin(Eigen::Vector4f(-10, -10, 0, 1.0));
    crop_box.setMax(Eigen::Vector4f(10, 10, 1, 1.0));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    crop_box.filter(*cropped_cloud);

    // print corrdinate of the cropped point cloud
    // for(int i = 0; i < cropped_cloud->points.size(); i++){
    //     RCLCPP_INFO(this->get_logger(), "cropped_cloud: %f, %f, %f", cropped_cloud->points[i].x, cropped_cloud->points[i].y, cropped_cloud->points[i].z);
    // }

    // fit a plane to the cropped point cloud
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // fit the plane
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE); // fit a plane
    seg.setMethodType(pcl::SAC_RANSAC); // RANSAC algorithm
    seg.setDistanceThreshold(0.01); // distance threshold for plane fitting
    seg.setInputCloud(cropped_cloud);
    seg.segment(*inliers, *coefficients);

    // print coefficients of the plane
    // RCLCPP_INFO(this->get_logger(), "Plane coefficients: %f, %f, %f, %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

    // create a point cloud with only the inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cropped_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_inliers);

    // RCLCPP_INFO(this->get_logger(), "cloud_inliers size: %d", cloud_inliers->points.size());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_local_map(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> crop_box2;
    crop_box2.setInputCloud(transformed_cloud);
    // crop_box2.setMin(Eigen::Vector4f(-10, 0.7-tool_height_wrt_base_link_, 0, 1.0));
    crop_box2.setMin(Eigen::Vector4f(-10, 0.4, 0, 1.0));
    crop_box2.setMax(Eigen::Vector4f(10, 10, 3, 1.0));
    // crop_box2.setMin(Eigen::Vector4f(-10, -10, -10, 1.0));
    // crop_box2.setMax(Eigen::Vector4f(10, 10, 10, 1.0));
    crop_box2.filter(*cloud_local_map);

    // RCLCPP_INFO(this->get_logger(), "cloud_local_map size: %d", cloud_local_map->points.size());

    // print the min and max x,y,z of the inliers
    double min_x = 1000, min_y = 1000, min_z = 1000, max_x = -1000, max_y = -1000, max_z = -1000;
    for(int i = 0; i < cloud_inliers->points.size(); i++){
        if(cloud_inliers->points[i].x < min_x){
            min_x = cloud_inliers->points[i].x;
        }
        if(cloud_inliers->points[i].y < min_y){
            min_y = cloud_inliers->points[i].y;
        }
        if(cloud_inliers->points[i].z < min_z){
            min_z = cloud_inliers->points[i].z;
        }
        if(cloud_inliers->points[i].x > max_x){
            max_x = cloud_inliers->points[i].x;
        }
        if(cloud_inliers->points[i].y > max_y){
            max_y = cloud_inliers->points[i].y;
        }
        if(cloud_inliers->points[i].z > max_z){
            max_z = cloud_inliers->points[i].z;
        }
    }
    // RCLCPP_INFO(this->get_logger(), "min_x: %f, min_y: %f, min_z: %f, max_x: %f, max_y: %f, max_z: %f", min_x, min_y, min_z, max_x, max_y, max_z);


    Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
    // transform_3.translation() << 0, 0.0, 0.0;
    // transform_3.rotate(Eigen::AngleAxisf(-robot_yaw, Eigen::Vector3f::UnitY()));
    // transform_3.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitX()));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_transformed (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud(*cloud_local_map, *cloud_plane_transformed, transform_3);

    // RCLCPP_INFO(this->get_logger(), "cloud_plane_transformed size: %d", cloud_plane_transformed->points.size());

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

    // transform coefficients by roll, pitch and yaw
    // RCLCPP_INFO(this->get_logger(), "initial: Plane coefficients: %f, %f, %f, %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
    coefficients->values[2] = -coefficients->values[1]*sin(pitch);
    coefficients->values[1] = -coefficients->values[1]*cos(pitch);

    // array with double to store elevation vlaues
    double elevation_values[global_map_.info.width*global_map_.info.height];
    double density_values[global_map_.info.width*global_map_.info.height];
    for(int i = 0; i < global_map_.info.width*global_map_.info.height; i++){
            elevation_values[global_map_.info.width*global_map_.info.height] = 0;
            density_values[global_map_.info.width*global_map_.info.height] = 0;
    }
    for(int i = 0; i < cloud_plane_transformed->points.size(); i++){
        float x_part = (cloud_plane_transformed->points[i].x*cos(robot_yaw) - (cloud_plane_transformed->points[i].z-0.4)*sin(robot_yaw));
        float y_part = (cloud_plane_transformed->points[i].x*sin(robot_yaw) + (cloud_plane_transformed->points[i].z-0.4)*cos(robot_yaw));
        int col_x = int((pose_x+x_part)/0.05);
        int row_y = int((pose_y+y_part)/0.05);
        // clip it by 0 and 159
        col_x = std::max(0, std::min(col_x, (int)global_map_.info.width));
        row_y = std::max(0, std::min(row_y, (int)global_map_.info.height));
        int global_idx = col_x + row_y*global_map_.info.width;
        global_idx = (global_idx % (global_map_.info.width*global_map_.info.height) + (global_map_.info.width*global_map_.info.height)) % (global_map_.info.width*global_map_.info.height);
        // double projected_y = -(coefficients->values[3]+coefficients->values[0]*cloud_plane_transformed->points[i].x+coefficients->values[2]*cloud_plane_transformed->points[i].z)/coefficients->values[1];
        double projected_d = (coefficients->values[0]*cloud_plane_transformed->points[i].x+coefficients->values[1]*cloud_plane_transformed->points[i].y+coefficients->values[2]*cloud_plane_transformed->points[i].z+coefficients->values[3]);
        // double elev = (cloud_plane_transformed->points[i].y-projected_y+pose_z);
        // RCLCPP_INFO(this->get_logger(), "elev: %f", elev);
        elevation_values[global_idx] += 100*(projected_d+pose_z-1.6);
        density_values[global_idx] += 1;
    }

    // RCLCPP_INFO(this->get_logger(), "local_elevation_map2 size: %d", local_elevation_map.data.size());

    // where pd_map is not 0, divide local_elevation_map by pd_map
    for(int i = 0; i < local_elevation_map.info.width*local_elevation_map.info.height; i++){
        if(density_values[i] > 0){
            global_map_.data[i] = int(elevation_values[i]/density_values[i]);
            // RCLCPP_INFO(this->get_logger(), "global_map_.data[i]: %d", global_map_.data[i]);
        }
    }



    // print the plane coefficients
    // RCLCPP_INFO(this->get_logger(), "Corrected: Plane coefficients: %f, %f, %f, %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

    double height_correction = -(coefficients->values[3]+0.4*coefficients->values[2])/coefficients->values[1] -0.63;
    // RCLCPP_INFO(this->get_logger(), "height_correction: %f", height_correction);

    // pcl_ros::transformPointCloud(*cloud, *cloud, transform);
    // tf2::doTransform(*cloud, *cloud, transform);

    // Convert the cropped point cloud back to a PointCloud2 message
    sensor_msgs::msg::PointCloud2 result_msg;
    pcl::toROSMsg(*cloud_local_map, result_msg);

    // Publish the transformed message
    publisher_pc_->publish(result_msg);
    publisher_global_map_->publish(global_map_);

    // publish tool height
    if(tool_height_wrt_base_link_ != -1000){
        std_msgs::msg::Float32 tool_height_msg;
        tool_height_msg.data = tool_height_wrt_base_link_ + height_correction;
        publisher_tool_height_->publish(tool_height_msg);
    }
  }





void GlobalMap::topic_callback_current_pose(const nav_msgs::msg::Odometry::SharedPtr msg){
    pose_x = msg->pose.pose.position.x + 6.6 + 2 + 2;
    pose_y = msg->pose.pose.position.y - 8.3 + 4;
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
    // print min and max x,y
    // RCLCPP_INFO(this->get_logger(), "min_x: %f, min_y: %f, max_x: %f, max_y: %f", min_x, min_y, max_x, max_y);
    // RCLCPP_INFO(this->get_logger(), "pose_x: %f, pose_y: %f", pose_x, pose_y);
    // pose_x = 0;
    // pose_y = 0;

    // from x,y,z,w to yaw
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(robot_roll, robot_pitch, robot_yaw);
    robot_yaw -= 1.5708;

    // print roll, pitch, yaw
    // RCLCPP_INFO(this->get_logger(), "roll: %f, pitch: %f, yaw: %f", robot_roll, robot_pitch, robot_yaw);

    // RCLCPP_INFO(this->get_logger(), "pose_x: %f, pose_y: %f, yaw: %f", pose_x, pose_y, yaw);
}

void GlobalMap::topic_callback_local_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    // RCLCPP_INFO(this->get_logger(), "Got local map");
    local_map_ = *msg;

    // transform occupancy grid using tf2
    // tf2_ros::Buffer tf_buffer(this->get_clock());
    // tf2_ros::TransformListener tfListener(tf_buffer);
    geometry_msgs::msg::TransformStamped transform;

    try{
      transform = tf_buffer_->lookupTransform("base_link", "base_link",tf2::TimePointZero, tf2::durationFromSec(1));
        // display transformStamped
        // RCLCPP_INFO(this->get_logger(), "transformStamped: %f", transformStamped);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "%s",ex.what());
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        return;
    }
    // RCLCPP_INFO(this->get_logger(), "Translation: %f, %f, %f", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
    // RCLCPP_INFO(this->get_logger(), "Rotation: %f, %f, %f, %f", transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);

    // pi/2
    // transformStamped.transform.rotation.x = 1.5;

    // nav_msgs::msg::OccupancyGrid local_map_transformed;
    // local_map_transformed.header.frame_id = "map";
    // copy local map to local map transformed
    // local_map_transformed.data = local_map_.data;
    // // apply transform to local map
    // tf2::doTransform(*msg, local_map_transformed, transform);


    // displace local map by (pose_x,pose_y) and copy it to global map
    for(int i = 0; i < local_map_.info.width; i++){
        for(int j = 0; j < local_map_.info.height; j++){

            // when yaw is 0
            // int global_map_idx = i+pose_x + (j+pose_y+global_map_.info.width)*global_map_.info.width;
            int global_map_idx = i+pose_x + (j+pose_y)*global_map_.info.width;

            // compute global map index when yaw is not 0 and it is radians
            // int global_map_idx = (int)(i*cos(yaw) - j*sin(yaw) + pose_x + (i*sin(yaw) + j*cos(yaw) + pose_y + global_map_.info.width)*global_map_.info.width);
            // int global_map_idx = pose_x + i*cos(yaw) - j*sin(yaw) + (pose_y + i*sin(yaw) + j*cos(yaw))*global_map_.info.width;

            // RCLCPP_INFO(this->get_logger(), "global_map_idx: %d, global_map_idx_og: %d", global_map_idx, global_map_idx_og);
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
    global_map_.data[0] = -100;

    publisher_global_map_->publish(global_map_);
}