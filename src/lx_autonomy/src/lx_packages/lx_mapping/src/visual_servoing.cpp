#include "lx_mapping/visual_servoing.hpp"
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

VisualServoing::VisualServoing() : Node("visual_servoing_node")
{   
    // set false for dry runs, set true for commands and to publish debug data
    debug_mode_ = true;
    int scale = 10;

    auto qos = rclcpp::SensorDataQoS();

    // not defined
    tool_height_wrt_base_link_ = 1000;
    
    subscription_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "camera/depth/color/points", 10, std::bind(&VisualServoing::topic_callback_pc, this, _1)); //subscribes to the point cloud topic at 1Hz

    subscription_aruco_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/aruco_poses", 10, std::bind(&VisualServoing::topic_callback_aruco_poses, this, _1)); //subscribes to the aruco markers topic at 10Hz

    publisher_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lx_mapping/vs_pc", 10);
    
    publisher_vs_error_ = this->create_publisher<std_msgs::msg::Float32>("lx_mapping/visual_servoing_error", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(100000000));    
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}

void VisualServoing::topic_callback_aruco_poses(const geometry_msgs::msg::PoseArray::SharedPtr msg){
   if(msg->poses.size()>0){
        geometry_msgs::msg::Pose aruco_pose = msg->poses[0];
        tool_height_wrt_base_link_ = -(aruco_pose.position.y* 0.7349 + aruco_pose.position.z* 0.2389 -0.2688)/0.6346;
   }
}

void VisualServoing::topic_callback_pc(const sensor_msgs::msg::PointCloud2::SharedPtr msg)  {

    if(tool_height_wrt_base_link_<0.2){
        RCLCPP_INFO(this->get_logger(), "VISUAL SERVOING DEACTIVATED, tool_height_wrt_base_link_: %f", tool_height_wrt_base_link_);
        return;
    }

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
    // RCLCPP_INFO(this->get_logger(), "roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);

    // get xyz translation from cam2map_transform
    double x = cam2map_transform.transform.translation.x;
    double y = cam2map_transform.transform.translation.y;
    double z = cam2map_transform.transform.translation.z;
    // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", x, y, z);

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
    pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, transform_2);

    // print tool height wrt base link
    RCLCPP_INFO(this->get_logger(), "tool_height_wrt_base_link_: %f", tool_height_wrt_base_link_);


    // MODE: TRAVERSAL
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(transformed_cloud);
    if(tool_height_wrt_base_link_ == 1000)
        crop_box.setMin(Eigen::Vector4f(-0.2, 0.35, -10, 1.0));
    else
        crop_box.setMin(Eigen::Vector4f(-0.2, 0.8-tool_height_wrt_base_link_, 0, 1.0));
    crop_box.setMax(Eigen::Vector4f(0.6, 1 ,2, 1.0));

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
            if(median_vec[idx]<max_elev && row[idx].size()>1){
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

    sensor_msgs::msg::PointCloud2 result_msg;
    pcl::toROSMsg(*cropped_cloud_target_berm, result_msg);

    publisher_pc_->publish(result_msg);
  }