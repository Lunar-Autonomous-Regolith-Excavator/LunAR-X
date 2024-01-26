/* Author: Anish Senathi
 * Subscribers:
 *    - /tool_height: [StdMsgs/Float64] Height of the tool wrt base link
 *    - /camera/depth/color/points: [SensorMsgs/PointCloud2] Point cloud from the camera
 * Publishers:
 *    - /mapping/transformed_pointcloud: [SensorMsgs/PointCloud2] Transformed point cloud
 *    - /mapping/pcl_ground_height: [StdMsgs/Float64] Ground height
 *    - /mapping/ground_pointcloud: [SensorMsgs/PointCloud2] Ground point cloud
 * Services:
 *    - /mapping/pcl_ground_height_srv: [LxMsgs/PclGroundHeight] Service to get the ground height from the tool for AutoDig control
 *
 * - Summary
 * - Takes in the point cloud from the camera and transforms it to the base link frame
 * - Crops the point cloud to the desired region and publishes it
 * - Publishes the ground height and the ground point cloud if needed
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
    ground_height_publisher_ = this->create_publisher<std_msgs::msg::Float64>("mapping/pcl_ground_height", 10);
    ground_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapping/ground_pointcloud", 10);
    
    // Service
    pcl_ground_height_service_ = this->create_service<lx_msgs::srv::PclGroundHeight>("mapping/pcl_ground_height_srv", 
                                                                                        std::bind(&PointCloudHandler::pclGroundHeightCallback, this, std::placeholders::_1, std::placeholders::_2));
    
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

void PointCloudHandler::pclGroundHeightCallback(const std::shared_ptr<lx_msgs::srv::PclGroundHeight::Request> req,
                                                const std::shared_ptr<lx_msgs::srv::PclGroundHeight::Response> res){
    this->need_ground_height_ = req->need_height;
    res->success = true;
}

void PointCloudHandler::processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    if(this->cam2map_transform.header.frame_id == ""){
        // Lookup the transform from the sensor frame to the target frame
        try
        {
            cam2map_transform = tf_buffer_->lookupTransform("base_link","camera_depth_optical_frame",tf2::TimePointZero, tf2::durationFromSec(1));
            RCLCPP_INFO(this->get_logger(), "Transform found");
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for transform... %s", ex.what());
            return;
        }
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

    Eigen::Affine3f affine_transform = Eigen::Affine3f::Identity();
    affine_transform.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    affine_transform.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
    affine_transform.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
    affine_transform.translation() << cam2map_transform.transform.translation.x, cam2map_transform.transform.translation.y, cam2map_transform.transform.translation.z;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, affine_transform);

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
        // When no tool height has been received, use a default value
        cropFilter.setMax(Eigen::Vector4f(1000, 1000, 0.28, 1.0));
    }
    else{
        // Crop region above the tool
        cropFilter.setMax(Eigen::Vector4f(1000, 1000, tool_height_wrt_base_link_-CROP_DISTANCE_FROM_TOP_M, 1.0));
    }
    cropFilter.setMin(Eigen::Vector4f(-1000, -1000, -20.0, 1.0));
    cropFilter.filter(*result_cloud);

    if(this->need_ground_height_){
        // crop the point cloud to the desired region (x_min = 0.7, xmax=1.1, y_min=0, y_max=0.5)

        pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_ground (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::CropBox<pcl::PointXYZ> cropFilterGround;
        cropFilterGround.setInputCloud(result_cloud);
        cropFilterGround.setMax(Eigen::Vector4f(1.0, 0.2, 1000, 1.0));
        cropFilterGround.setMin(Eigen::Vector4f(0.7, -0.2, -1000, 1.0));
        cropFilterGround.filter(*cropped_cloud_ground);

        // get average z value of cropped point cloud and publish it
        double avg_z = 0.0;
        if (cropped_cloud_ground->points.size() != 0)
        {
            for(size_t i=0; i<cropped_cloud_ground->points.size(); i++){
                avg_z += cropped_cloud_ground->points[i].z;
            }
            avg_z /= cropped_cloud_ground->points.size();
        }
        std_msgs::msg::Float64 avg_z_msg;
        avg_z_msg.data = exp_height_filter_.getValue(avg_z);
        ground_height_publisher_->publish(avg_z_msg);
        //publish the pointcloud if debug mode is on
        if (debug_mode_)
        {
            sensor_msgs::msg::PointCloud2 cropped_cloud_msg;
            pcl::toROSMsg(*cropped_cloud_ground, cropped_cloud_msg);
            cropped_cloud_msg.header.frame_id = "base_link";
            ground_pointcloud_publisher_->publish(cropped_cloud_msg);
        }
    }

    sensor_msgs::msg::PointCloud2 result_msg;
    pcl::toROSMsg(*result_cloud, result_msg);   
    result_msg.header.frame_id = "base_link"; 
    transformed_pointcloud_publisher_->publish(result_msg);
}