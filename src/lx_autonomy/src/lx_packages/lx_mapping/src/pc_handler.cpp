#include "lx_mapping/pc_handler.hpp"


const double MAP_DIMENSION = 8.0;
const double MAP_RESOLUTION = 0.05;
PointCloudHandler::PointCloudHandler() : Node("pc_handler_node")
{   
    debug_mode_ = true;

    auto qos = rclcpp::SensorDataQoS();

    std::placeholders::_2;

    publisher_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lx_mapping/transformed_pc_", 10);
    
    got_pointcloud = false;

    service_pc_ = this->create_service<lx_msgs::srv::Map>("lx_mapping/pc_handler", std::bind(&PointCloudHandler::startStopPCHCallback, this, std::placeholders::_1, std::placeholders::_2));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(100000000));    
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    min_x = 1000, min_y = 1000, max_x = -1000, max_y = -1000, min_z = 1000, max_z = -1000;
    min_col = 1000, min_row = 1000, max_col = -1000, max_row = -1000;
    this->tool_height_wrt_base_link_ = 1000.0;
}




void PointCloudHandler::startStopPCHCallback(const std::shared_ptr<lx_msgs::srv::Map::Request> request,
    std::shared_ptr<lx_msgs::srv::Map::Response> response){
        // print the request message
        RCLCPP_INFO(this->get_logger(), "Incoming request\nstart: %d", request->start);
        if(request->start){
            this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "camera/depth/color/points", 10, std::bind(&PointCloudHandler::transform_pc_cam2map, this, _1)); //subscribes to the point cloud topic at 1Hz

        }
        else{
            this->subscription_pc_.reset();
        }
        response->success = true;
    }







void PointCloudHandler::topic_callback_get_tool_height(const geometry_msgs::msg::PoseArray::SharedPtr msg){
   if(msg->poses.size()>0){
        geometry_msgs::msg::Pose aruco_pose = msg->poses[0];
        tool_height_wrt_base_link_ = -(aruco_pose.position.y* 0.7349 + aruco_pose.position.z* 0.2389 -0.2688)/0.6346;
   }
}








void PointCloudHandler::transform_pc_cam2map(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    // Lookup the transform from the sensor frame to the target frame
    geometry_msgs::msg::TransformStamped cam2map_transform;
    try
    {
      cam2map_transform = tf_buffer_->lookupTransform("moonyard","camera_depth_optical_frame",tf2::TimePointZero, tf2::durationFromSec(1));
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
    transform_2.translation() << cam2map_transform.transform.translation.x, cam2map_transform.transform.translation.y, cam2map_transform.transform.translation.z;
    transform_2.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    transform_2.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
    transform_2.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, transform_2);

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
    if(debug_mode_){
        RCLCPP_INFO(this->get_logger(), "Model coefficients: %f %f %f %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
    }

    //crop the point cloud to the desired region
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> cropFilter;
    cropFilter.setInputCloud(transformed_cloud);
    if(this->tool_height_wrt_base_link_ == 1000.0){
        cropFilter.setMin(Eigen::Vector4f(-1000, -1000, coefficients->values[3]-0.45, 1.0));
    }
    else{
        cropFilter.setMin(Eigen::Vector4f(-1000, -1000, coefficients->values[3]-tool_height_wrt_base_link_, 1.0));
    }
    cropFilter.setMax(Eigen::Vector4f(1000, 1000, 2.0, 1.0));
    cropFilter.filter(*result_cloud);

    sensor_msgs::msg::PointCloud2 result_msg;
    pcl::toROSMsg(*result_cloud, result_msg);   
    result_msg.header.frame_id = "moonyard"; 
    publisher_pc_->publish(result_msg);
}