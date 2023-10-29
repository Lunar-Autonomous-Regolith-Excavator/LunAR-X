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
 * - Add special zones from user input
 * - Test with rover
 * - Check compatibility with planner
 * */


#include "lx_mapping/world_model.hpp"


WorldModel::WorldModel() : Node("world_model_node")
{   
    debug_mode_ = false;

    // Setup Communications
    setupCommunications();

    // Setup Maps
    configureMaps();

    // Initializing occupancy grid
    this->global_map_.data.resize(global_map_.info.width*global_map_.info.height);
    for(int i = 0; i < this->global_map_.info.width*this->global_map_.info.height; i++){
        this->global_map_.data[i] = 0;
    }
}

void WorldModel::setupCommunications(){

    // Publishers
    global_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("mapping/global_map", 10);
    world_model_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("mapping/world_model", 10);

    // Servers
    map_switch_server_ = this->create_service<lx_msgs::srv::Switch>("mapping/map_switch", 
                                                            std::bind(&WorldModel::mapSwitchCallback, 
                                                            this, std::placeholders::_1, std::placeholders::_2));
}

void WorldModel::configureMaps(){
    // Configuring occupancy grid

    this->global_map_.header.frame_id = "moonyard";
    this->global_map_.info.resolution = MAP_RESOLUTION;
    this->global_map_.info.width = MAP_DIMENSION/MAP_RESOLUTION;
    this->global_map_.info.height = MAP_DIMENSION/MAP_RESOLUTION;
    this->global_map_.info.origin.position.x = -MAP_DIMENSION/2.0;
    this->global_map_.info.origin.position.y = -MAP_DIMENSION/2.0;

    elevation_costmap_.header.frame_id = "moonyard";
    elevation_costmap_.info = global_map_.info;

    slope_costmap_.header.frame_id = "moonyard";
    slope_costmap_.info = global_map_.info;

    berm_costmap_.header.frame_id = "moonyard";
    berm_costmap_.info = global_map_.info;

    zone_costmap_.header.frame_id = "moonyard";
    zone_costmap_.info = global_map_.info;

    world_model_.header.frame_id = "moonyard";
    world_model_.info = global_map_.info;
}

void WorldModel::mapSwitchCallback(const std::shared_ptr<lx_msgs::srv::Switch::Request> req,
                                                std::shared_ptr<lx_msgs::srv::Switch::Response> res){
    if(req->switch_state){
        // Subscribe to the point cloud topic
        transformed_pcl_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("mapping/transformed_pointcloud", 10, 
                                                                                    std::bind(&WorldModel::transformedPCLCallback, this, _1));
    }
    else{
        transformed_pcl_subscriber_.reset();
    }
    res->success = true;
}

void WorldModel::transformedPCLCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    fuse_map_thread_ = std::thread(std::bind(&WorldModel::fuseMap, this, msg));

    // Have to detach thread before it goes out of scope
    fuse_map_thread_.detach();
}

void WorldModel::fuseMap(const sensor_msgs::msg::PointCloud2::SharedPtr msg)  {

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
    
    global_map_publisher_->publish(global_map_);

    buildWorldModel();
}


void WorldModel::buildWorldModel(){

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
    world_model_publisher_->publish(world_model_);
}