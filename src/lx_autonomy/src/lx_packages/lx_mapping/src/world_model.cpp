#include "lx_mapping/world_model.hpp"

WorldModel::WorldModel() : Node("world_model_node")
{   
    debug_mode_ = true;

    auto qos = rclcpp::SensorDataQoS();

    std::placeholders::_2;

    subscription_global_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "lx_mapping/global_map", 10, std::bind(&WorldModel::topic_callback_global_map, this, _1)); //subscribes to the point cloud topic at 1Hz

    publisher_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lx_mapping/transformed_pc_", 10);
    publisher_global_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("lx_mapping/global_map", 10);
    publisher_world_model_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("lx_mapping/world_model", 10);
    
    got_pointcloud = false;

    service_map_ = this->create_service<lx_msgs::srv::Map>("lx_mapping/start_stop_mapping", std::bind(&WorldModel::startStopMappingCallback, this, std::placeholders::_1, std::placeholders::_2));

    nav_msgs::msg::OccupancyGrid global_map_, elevation_costmap_,slope_costmap_, berm_costmap_, zone_costmap_, world_model_;

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


void WorldModel::topic_callback_aruco_poses(const geometry_msgs::msg::PoseArray::SharedPtr msg){
   if(msg->poses.size()>0){
        geometry_msgs::msg::Pose aruco_pose = msg->poses[0];
        tool_height_wrt_base_link_ = -(aruco_pose.position.y* 0.7349 + aruco_pose.position.z* 0.2389 -0.2688)/0.6346;
   }
}




void WorldModel::transform_pc_cam2map(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
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
    // RCLCPP_INFO(this->get_logger(), "roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);

    double robot_x = cam2map_transform.transform.translation.x;
    double robot_y = cam2map_transform.transform.translation.y;
    double robot_z = cam2map_transform.transform.translation.z;
    // RCLCPP_INFO(this->get_logger(), "robot_x: %f, robot_y: %f, robot_z: %f, pose_x: %f, pose_y: %f, pose_z: %f", robot_x, robot_y, robot_z, pose_x, pose_y, pose_z);

    // RCLCPP_INFO(this->get_logger(), "min_x: %f, max_x: %f, min_y: %f, max_y: %f", min_x, max_x, min_y, max_y);

    // crop the point cloud
    sensor_msgs::msg::PointCloud2 cropped_msg;
    
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
    // transform_2.translation() << robot_x, robot_y, robot_z;
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
    seg.setModelType (pcl::SACMODEL_PLANE); // SACMODEL_PERPENDICULAR_PLANE
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud (transformed_cloud);
    seg.segment (*inliers, *coefficients);
    //print coefficients
    RCLCPP_INFO(this->get_logger(), "Model coefficients: %f %f %f %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

    sensor_msgs::msg::PointCloud2 result_msg;
    pcl::toROSMsg(*cloud_filtered, result_msg);    
    publisher_pc_->publish(result_msg);
}

void WorldModel::topic_callback_fuse_map(const sensor_msgs::msg::PointCloud2::SharedPtr msg)  {
    transform_pc_cam2map(msg);
  }



void WorldModel::topic_callback_current_pose(const nav_msgs::msg::Odometry::SharedPtr msg){
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