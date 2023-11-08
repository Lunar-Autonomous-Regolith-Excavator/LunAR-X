/* Author: Anish Senathi
 * Subscribers:
 *    - /topic: description
 * Publishers:
 *    - /topic: description
 * Services:
 *    - /name (type): description
 * Actions:
 *    - /name (type): description
 *
 * - Summary
 * 
 * TODO
 * - Add Documentation
 * - Test with rover
 * - Check compatibility with planner
 * */


#include "lx_mapping/pc_handler.hpp"

PointCloudHandler::PointCloudHandler() : Node("pc_handler_node")
{       
    // Setup Communications
    setupCommunications();

    this->tool_height_wrt_base_link_ = 1000.0;

    RCLCPP_INFO(this->get_logger(), "Point Cloud Handler initialized");
}


void PointCloudHandler::setupCommunications(){
    // Subscribers
    tool_height_subscriber_ = this->create_subscription<std_msgs::msg::Float64>("tool_height", 10, 
                                                                                        std::bind(&PointCloudHandler::toolHeightCallback, this, std::placeholders::_1));
    pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("camera/depth/color/points", 10, 
                                                                                        std::bind(&PointCloudHandler::processPointCloud, this, std::placeholders::_1));
    // Publishers
    transformed_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapping/transformed_pointcloud", 10);
    // Transform Listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(100000000));    
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}


void PointCloudHandler::toolHeightCallback(const std_msgs::msg::Float64::SharedPtr msg){
   tool_height_wrt_base_link_ = msg->data;
}


void PointCloudHandler::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    pointcloud_thread_ = std::thread(std::bind(&PointCloudHandler::processPointCloud, this, msg));

    pointcloud_thread_.detach();
}


void PointCloudHandler::processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    // Lookup the transform from the sensor frame to the target frame
    geometry_msgs::msg::TransformStamped cam2map_transform;
    try
    {
      cam2map_transform = tf_buffer_->lookupTransform("base_link","camera_depth_optical_frame",tf2::TimePointZero, tf2::durationFromSec(1));
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
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    //remove noise from the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);  

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    transform_2.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
    transform_2.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
    transform_2.translation() << cam2map_transform.transform.translation.x, cam2map_transform.transform.translation.y, cam2map_transform.transform.translation.z;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_1 (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud(*cloud_filtered, *transformed_cloud_1, transform_2);

    Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
    // transform_3.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    // transform_3.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
    // transform_3.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
    // transform_3.translation() << cam2map_transform.transform.translation.x, cam2map_transform.transform.translation.y, cam2map_transform.transform.translation.z;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud(*transformed_cloud_1, *transformed_cloud, transform_3);

    if(debug_mode_){
        // find the best fit plane
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // find coefficients
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud (transformed_cloud);
        seg.segment (*inliers, *coefficients);

        RCLCPP_INFO(this->get_logger(), "Model coefficients: %f %f %f %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
    }

    //crop the point cloud to the desired region
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> cropFilter;
    cropFilter.setInputCloud(transformed_cloud);
    // TODO: check for z direction
    if(this->tool_height_wrt_base_link_ == 1000.0){
        cropFilter.setMax(Eigen::Vector4f(1000, 1000, 0.3, 1.0));
    }
    else{
        cropFilter.setMax(Eigen::Vector4f(1000, 1000, tool_height_wrt_base_link_-0.2, 1.0));
    }
    cropFilter.setMin(Eigen::Vector4f(-1000, -1000, -20.0, 1.0));
    cropFilter.filter(*result_cloud);

    sensor_msgs::msg::PointCloud2 result_msg;
    pcl::toROSMsg(*result_cloud, result_msg);   
    result_msg.header.frame_id = "base_link"; 
    transformed_pointcloud_publisher_->publish(result_msg);
}