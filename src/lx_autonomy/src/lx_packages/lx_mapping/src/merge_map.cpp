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
    tool_height_wrt_base_link_ = -1000;
    min_x=1000.0; min_y=1000.0; max_x=-1000.0; max_y=-1000.0;

    double robot_roll = 0;
    double robot_pitch = 0;
    double robot_yaw = 0;

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

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(100000000));    
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
    geometry_msgs::msg::TransformStamped cam2map_transform;
    try
    {
      cam2map_transform = tf_buffer_->lookupTransform("camera_link", "base_link",tf2::TimePointZero, tf2::durationFromSec(1));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", ex.what());
      return;
    }
    
    // convert x,y,z,w to roll, pitch, yaw
    tf2::Quaternion q(cam2map_transform.transform.rotation.x, cam2map_transform.transform.rotation.y, cam2map_transform.transform.rotation.z, cam2map_transform.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    RCLCPP_INFO(this->get_logger(), "roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);

    // get xyz translation from cam2map_transform
    double x = cam2map_transform.transform.translation.x;
    double y = cam2map_transform.transform.translation.y;
    double z = cam2map_transform.transform.translation.z;
    RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", x, y, z);

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
    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform_2);

    // print tool height wrt base link
    RCLCPP_INFO(this->get_logger(), "tool_height_wrt_base_link_: %f", tool_height_wrt_base_link_);


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

    // MODE: TRAVERSAL TO AUTO-DUMP (APPROACHING BERM)
    pcl::CropBox<pcl::PointXYZ> crop_box_2;
    crop_box.setInputCloud(cropped_cloud_local_map);
    crop_box.setMin(Eigen::Vector4f(-0.2, -1, 0, 1.0));
    crop_box.setMax(Eigen::Vector4f(0.6, 1 , 2, 1.0));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_target_berm(new pcl::PointCloud<pcl::PointXYZ>);
    crop_box.filter(*cropped_cloud_target_berm);

    // 2D vector of length 80x200
    // define a vector
    // std::vector< std::vector<int> > vec(4, std::vector<int>(4));
    std::vector<double> vec(1,100);
    int resolution = 5;
    int num_rows = 200/resolution;
    int num_cols = 80/resolution;
    std::vector<std::vector<double>> row ((num_cols+1)*num_rows,vec);
    RCLCPP_INFO(this->get_logger(), "D");

    for(int i = 0; i < cropped_cloud_target_berm->points.size(); i++){
        int idx = int(num_rows*100*(cropped_cloud_target_berm->points[i].x+0.2)/resolution) + int((100*cropped_cloud_target_berm->points[i].z)/resolution);
        row[idx].push_back(cropped_cloud_target_berm->points[i].y);
    }
    RCLCPP_INFO(this->get_logger(), "E");

    std::vector<double> median_vec(num_cols*num_rows,0);
    std::vector<double> peak_y(num_cols,0);
    std::vector<double> peak_z(num_cols,0);
    RCLCPP_INFO(this->get_logger(), "A");

    for(int i=0;i<num_cols;i++){
        double max_elev = 1000;
        for(int j=0;j<num_rows;j++){
            int idx = num_rows*i+j;
            std::sort(row[idx].begin(), row[idx].end());
            median_vec[idx] = row[idx][row[idx].size()/2];
            if(median_vec[idx]<max_elev){
                max_elev = median_vec[idx];
                peak_z[i] = max_elev;
                peak_y[i] = j*resolution/100.0;
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "C");

    for(int i=0;i<num_cols;i++){
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", i*resolution/100.0,peak_y[i],peak_z[i]);
    }



    // double median;
    // int size = vec.size();

    // if(size%2==1){
    //     median = vec[size / 2];
    // }
    // else{
    //     median = (vec[size / 2] + vec[size / 2 - 1])/2.0;
    // }

    // // print median
    // RCLCPP_INFO(this->get_logger(), "median: %f", median);

    // print corrdinate of the cropped point cloud
    // for(int i = 0; i < cropped_cloud->points.size(); i++){
    //     RCLCPP_INFO(this->get_logger(), "cropped_cloud: %f, %f, %f", cropped_cloud->points[i].x, cropped_cloud->points[i].y, cropped_cloud->points[i].z);
    // }
/*
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
    RCLCPP_INFO(this->get_logger(), "Plane coefficients: %f, %f, %f, %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

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
    RCLCPP_INFO(this->get_logger(), "initial: Plane coefficients: %f, %f, %f, %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
    coefficients->values[2] = -coefficients->values[1]*sin(pitch);
    coefficients->values[1] = -coefficients->values[1]*cos(pitch);

    // array with double to store elevation vlaues
    double elevation_values[global_map_.info.width*global_map_.info.height];
    double density_values[global_map_.info.width*global_map_.info.height];
    for(int i = 0; i < global_map_.info.width*global_map_.info.height; i++){
            elevation_values[global_map_.info.width*global_map_.info.height] = 0;
            density_values[global_map_.info.width*global_map_.info.height] = 0;
    }
    // RCLCPP_INFO(this->get_logger(), "pose_x: %f, pose_y: %f", pose_x, pose_y);
    int bins[100];
    for(int i = 0; i < 100; i++){
        bins[i] = 0;
    }
    for(int i = 0; i < cloud_plane_transformed->points.size(); i++){
        float x_part = (cloud_plane_transformed->points[i].x*cos(robot_yaw) - (cloud_plane_transformed->points[i].z-0.4)*sin(robot_yaw));
        float y_part = (cloud_plane_transformed->points[i].x*sin(robot_yaw) + (cloud_plane_transformed->points[i].z-0.4)*cos(robot_yaw));
        int col_x = int((pose_x+x_part)/0.05);
        int row_y = int((pose_y+y_part)/0.05);
        // RCLCPP_INFO(this->get_logger(), "pose_x: %f, pose_y: %f", pose_x, pose_y);
        // RCLCPP_INFO(this->get_logger(), "col_x: %d, row_y: %d", col_x, row_y);
        // clip it by 0 and 159
        // col_x = std::max(0, std::min(col_x, (int)global_map_.info.width));
        // row_y = std::max(0, std::min(row_y, (int)global_map_.info.height));
        int global_idx = col_x + row_y*global_map_.info.width;
        // get positive modulo of global_idx
        global_idx = (global_idx % (global_map_.info.width*global_map_.info.height));


        // global_idx = (global_idx % (global_map_.info.width*global_map_.info.height) + (global_map_.info.width*global_map_.info.height)) % (global_map_.info.width*global_map_.info.height);
        // double projected_y = -(coefficients->values[3]+coefficients->values[0]*cloud_plane_transformed->points[i].x+coefficients->values[2]*cloud_plane_transformed->points[i].z)/coefficients->values[1];
        // double projected_d = (coefficients->values[0]*cloud_plane_transformed->points[i].x+coefficients->values[1]*cloud_plane_transformed->points[i].y+coefficients->values[2]*cloud_plane_transformed->points[i].z+coefficients->values[3]);
        // double elev = (cloud_plane_transformed->points[i].y-projected_y+pose_z);
        // RCLCPP_INFO(this->get_logger(), "elev: %f", elev);
        double elev = (cloud_plane_transformed->points[i].y-pose_z);
        // elevation_values[global_idx] += 100*(projected_d+pose_z-1.6);
        elevation_values[global_idx] += 150*(elev+4);
        density_values[global_idx] += 1;

        bins[int(50*cloud_plane_transformed->points[i].z)] += 1;
    }

    int max_bin_idx = 0;
    int max_bin_val = 0;
    for(int i = 0; i < 100  ; i++){
        // RCLCPP_INFO(this->get_logger(), "bin %d: %d", i, bins[i]);
        if(bins[i] > max_bin_val){
            max_bin_val = bins[i];
            max_bin_idx = i;
        }
    }
    // do while loop
    int peak_idx = max_bin_idx;
    while(bins[peak_idx] > 20){
        peak_idx++;
    }
    float berm_peak_height = peak_idx*0.02 - 0.4;
    RCLCPP_INFO(this->get_logger(), "berm_peak_height: %f", berm_peak_height);

    if(tool_height_wrt_base_link_ != -1000){
        float height_diff = tool_height_wrt_base_link_ - berm_peak_height;
        RCLCPP_INFO(this->get_logger(), "height_diff: %f", height_diff);
    }


    // RCLCPP_INFO(this->get_logger(), "local_elevation_map2 size: %d", local_elevation_map.data.size());

    // where pd_map is not 0, divide local_elevation_map by pd_map
    if(robot_yaw!=0){
        for(int i = 0; i < local_elevation_map.info.width*local_elevation_map.info.height; i++){
            if(density_values[i] > 0){
                global_map_.data[i] = int(elevation_values[i]/density_values[i]) + 100;
                elevation_values[i] = 0;    
                density_values[i] = 0;
                // RCLCPP_INFO(this->get_logger(), "global_map_.data[i]: %d", global_map_.data[i]);
            }
        }
    }


    // print the plane coefficients
    // RCLCPP_INFO(this->get_logger(), "Corrected: Plane coefficients: %f, %f, %f, %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

    double height_correction = -(coefficients->values[3]+0.4*coefficients->values[2])/coefficients->values[1] -0.63;
    // RCLCPP_INFO(this->get_logger(), "height_correction: %f", height_correction);

    // pcl_ros::transformPointCloud(*cloud, *cloud, transform);
    // tf2::doTransform(*cloud, *cloud, transform);
*/
    // Convert the cropped point cloud back to a PointCloud2 message
    sensor_msgs::msg::PointCloud2 result_msg;
    pcl::toROSMsg(*cropped_cloud_target_berm, result_msg);

    // Publish the transformed message
    publisher_pc_->publish(result_msg);
    // publisher_global_map_->publish(global_map_);

    // publish tool height
    // if(tool_height_wrt_base_link_ != -1000){
    //     std_msgs::msg::Float32 tool_height_msg;
    //     tool_height_msg.data = tool_height_wrt_base_link_ + height_correction;
    //     publisher_tool_height_->publish(tool_height_msg);
    // }
  }





void GlobalMap::topic_callback_current_pose(const nav_msgs::msg::Odometry::SharedPtr msg){
    pose_x = msg->pose.pose.position.x;
    pose_y = msg->pose.pose.position.y;
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

    // from x,y,z,w to yaw
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(robot_roll, robot_pitch, robot_yaw);
    robot_yaw -= 1.5708;
}