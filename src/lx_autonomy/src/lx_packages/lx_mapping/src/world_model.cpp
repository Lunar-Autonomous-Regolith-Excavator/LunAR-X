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
}








void WorldModel::startStopMappingCallback(const std::shared_ptr<lx_msgs::srv::Map::Request> request,
        std::shared_ptr<lx_msgs::srv::Map::Response> response){
    if(request->start){
        subscription_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "lx_mapping/transformed_pc_", 10, std::bind(&WorldModel::topic_callback_fuse_map, this, _1)); //subscribes to the point cloud topic at 1Hz
    
    }
    else{
        subscription_pc_.reset();
    }
    response->success = true;
}





void WorldModel::topic_callback_fuse_map(const sensor_msgs::msg::PointCloud2::SharedPtr msg)  {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_local_map(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cropped_cloud_local_map);  

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
            RCLCPP_INFO(this->get_logger(), "max: %f, min: %f, slope: %f", max_neighbour, min_neighbour, 10*(max_neighbour - min_neighbour));
        }
    }
    
    for(int i=0;i<world_model_.data.size();i++){
        world_model_.data[i] = std::max(elevation_costmap_.data[i], slope_costmap_.data[i]);
    }
    publisher_world_model_->publish(world_model_);
}