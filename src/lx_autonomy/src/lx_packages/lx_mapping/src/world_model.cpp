#include "lx_mapping/world_model.hpp"


const double MAP_DIMENSION = 8.0;
const double MAP_RESOLUTION = 0.05;
WorldModel::WorldModel() : Node("world_model_node")
{   
    debug_mode_ = false;

    auto qos = rclcpp::SensorDataQoS();

    std::placeholders::_2;

    subscription_global_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "lx_mapping/global_map", 10, std::bind(&WorldModel::topic_callback_global_map, this, _1)); //subscribes to the point cloud topic at 1Hz

    publisher_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lx_mapping/transformed_pc_", 10);
    publisher_global_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("lx_mapping/global_map", 10);
    publisher_world_model_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("lx_mapping/world_model", 10);
    
    got_pointcloud = false;

    service_map_ = this->create_service<lx_msgs::srv::Map>("lx_mapping/start_stop_mapping", std::bind(&WorldModel::startStopMappingCallback, this, std::placeholders::_1, std::placeholders::_2));

    // configuring occupancy grid
    this->global_map_.header.frame_id = "moonyard";
    this->global_map_.info.resolution = MAP_RESOLUTION;
    this->global_map_.info.width = MAP_DIMENSION/MAP_RESOLUTION;
    this->global_map_.info.height = MAP_DIMENSION/MAP_RESOLUTION;
    this->global_map_.info.origin.position.x = -MAP_DIMENSION/2.0;
    this->global_map_.info.origin.position.y = -MAP_DIMENSION/2.0;

    // initializing occupancy grid
    this->global_map_.data.resize(global_map_.info.width*global_map_.info.height);
    for(int i = 0; i < this->global_map_.info.width*this->global_map_.info.height; i++){
        this->global_map_.data[i] = 0;
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(100000000));    
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    min_x = 1000, min_y = 1000, max_x = -1000, max_y = -1000, min_z = 1000, max_z = -1000;
    min_col = 1000, min_row = 1000, max_col = -1000, max_row = -1000;
    this->tool_height_wrt_base_link_ = 1000.0;
}








void WorldModel::startStopMappingCallback(const std::shared_ptr<lx_msgs::srv::Map::Request> request,
        std::shared_ptr<lx_msgs::srv::Map::Response> response){
    if(request->start){
        subscription_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "camera/depth/color/points", 10, std::bind(&WorldModel::topic_callback_fuse_map, this, _1)); //subscribes to the point cloud topic at 1Hz
    }
    else{
        subscription_pc_.reset();
    }
    response->success = true;
}






void WorldModel::topic_callback_get_tool_height(const geometry_msgs::msg::PoseArray::SharedPtr msg){
   if(msg->poses.size()>0){
        geometry_msgs::msg::Pose aruco_pose = msg->poses[0];
        tool_height_wrt_base_link_ = -(aruco_pose.position.y* 0.7349 + aruco_pose.position.z* 0.2389 -0.2688)/0.6346;
   }
}








pcl::PointCloud<pcl::PointXYZ>::Ptr WorldModel::transform_pc_cam2map(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    // Lookup the transform from the sensor frame to the target frame
    geometry_msgs::msg::TransformStamped cam2map_transform;
    try
    {
      cam2map_transform = tf_buffer_->lookupTransform("moonyard","camera_depth_optical_frame",tf2::TimePointZero, tf2::durationFromSec(1));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", ex.what());
      return nullptr;
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
    return result_cloud;
}

void WorldModel::topic_callback_fuse_map(const sensor_msgs::msg::PointCloud2::SharedPtr msg)  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_local_map = transform_pc_cam2map(msg);

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
        int col_x = int(cropped_cloud_local_map->points[i].x / global_map_.info.resolution ) + global_map_.info.width/2;
        int row_y = int(cropped_cloud_local_map->points[i].y / global_map_.info.resolution ) + global_map_.info.height/2;

        col_x = std::min(std::max(col_x, 0), int(global_map_.info.width-1));
        row_y = std::min(std::max(row_y, 0), int(global_map_.info.height-1));

        int global_idx = col_x + row_y*global_map_.info.width;
        double elev = cropped_cloud_local_map->points[i].z;
        elevation_values[global_idx] += 200.0*(elev-1.4);
        density_values[global_idx] += 1.0;
    }

    for(int i = 0; i < global_map_.info.width*global_map_.info.height; i++){
        if(density_values[i] > 1.0){
            global_map_.data[i] = int(elevation_values[i]/density_values[i]);
        }
    }
    
    publisher_global_map_->publish(global_map_);
  }















void WorldModel::topic_callback_global_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    global_map_ = *msg;
    elevation_costmap_ = *msg;
    slope_costmap_ = *msg;
    world_model_ = *msg;
    for(int i = 0; i < elevation_costmap_.data.size(); i++){
        if (global_map_.data[i] != 0 && global_map_.data[i] < 20){
            elevation_costmap_.data[i] = 100 - global_map_.data[i];
        }        
        else{
            elevation_costmap_.data[i] = 0;
        }
    }
    // int neighbour_deltas[8] = [-1, 1, -global_map_.info.width, global_map_.info.width, -global_map_.info.width-1, -global_map_.info.width+1, global_map_.info.width-1, global_map_.info.width+1];
    int neighbour_deltas[8] = {-1, 1, -global_map_.info.width, global_map_.info.width, -global_map_.info.width-1, -global_map_.info.width+1, global_map_.info.width-1, global_map_.info.width+1};

    for(int i=0; i<slope_costmap_.data.size(); i++){
        double max_neighbour = -100, min_neighbour = 100;
        for(int j=0;j<8;j++){
            int neighbour_idx = i+neighbour_deltas[j];
            if(neighbour_idx < 0 || neighbour_idx > slope_costmap_.data.size()){
                continue;
            }
            else if(global_map_.data[neighbour_idx] == 0){
                continue;
            }   
            if(global_map_.data[i+neighbour_deltas[j]] > max_neighbour){
                max_neighbour = global_map_.data[i+neighbour_deltas[j]];
            }
            if(global_map_.data[i+neighbour_deltas[j]] < min_neighbour){
                min_neighbour = global_map_.data[i+neighbour_deltas[j]];
            }
        }
        if(max_neighbour == -100 || min_neighbour == 100){
            slope_costmap_.data[i] = 0;
        }
        else{
            slope_costmap_.data[i] = max_neighbour - min_neighbour;
        }
    }
    
    for(int i=0;i<world_model_.data.size();i++){
        world_model_.data[i] = std::max(elevation_costmap_.data[i], slope_costmap_.data[i]);
    }
    publisher_world_model_->publish(world_model_);
}