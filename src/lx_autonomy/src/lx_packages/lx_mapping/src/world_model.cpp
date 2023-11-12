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
    this->filtered_global_map_.data.resize(global_map_.info.width*global_map_.info.height);
    for(size_t i = 0; i < this->global_map_.info.width*this->global_map_.info.height; i++){
        this->global_map_.data[i] = 0;
        this->filtered_global_map_.data[i] = 0;
    }

    RCLCPP_INFO(this->get_logger(), "World Model initialized");
}

void WorldModel::setupCommunications(){

    // Publishers
    global_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("mapping/global_map", 10);
    world_model_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("mapping/world_model", 10);
    filtered_global_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("mapping/filtered_global_map", 10);

    // Servers
    map_switch_server_ = this->create_service<lx_msgs::srv::Switch>("mapping/map_switch", 
                                                            std::bind(&WorldModel::mapSwitchCallback, 
                                                            this, std::placeholders::_1, std::placeholders::_2));
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(100000000));    
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void WorldModel::configureMaps(){
    // Configuring occupancy grid

    this->global_map_.header.frame_id = "moonyard";
    this->global_map_.info.resolution = MAP_RESOLUTION;
    this->global_map_.info.width = MAP_DIMENSION/MAP_RESOLUTION;
    this->global_map_.info.height = MAP_DIMENSION/MAP_RESOLUTION;
    this->global_map_.info.origin.position.x = -MAP_DIMENSION/2.0;
    this->global_map_.info.origin.position.y = -MAP_DIMENSION/2.0;

    filtered_global_map_.header.frame_id = "moonyard";
    filtered_global_map_.info = global_map_.info;

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

    // Initialize bayes filter
    for(size_t i = 0; i < global_map_.info.width*global_map_.info.height; i++){
        BayesFilter bf;
        bayes_filter_.push_back(bf);
    }
}


void WorldModel::mapSwitchCallback(const std::shared_ptr<lx_msgs::srv::Switch::Request> req,
                                                std::shared_ptr<lx_msgs::srv::Switch::Response> res){
    if(req->switch_state){
        // Subscribe to the Point cloud topic
        transformed_pcl_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("mapping/transformed_pointcloud", 10, 
                                                                                    std::bind(&WorldModel::transformedPCLCallback, this, std::placeholders::_1));
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

pcl::PointCloud<pcl::PointXYZ>::Ptr WorldModel::transformMap(const sensor_msgs::msg::PointCloud2::SharedPtr msg)  {
    
    geometry_msgs::msg::TransformStamped base2moonyard_transform;
    try
    {
      base2moonyard_transform = tf_buffer_->lookupTransform("moonyard","base_link",tf2::TimePointZero, tf2::durationFromSec(10));
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", ex.what());

        return nullptr;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // convert x,y,z,w to roll, pitch, yaw
    tf2::Quaternion q(base2moonyard_transform.transform.rotation.x, base2moonyard_transform.transform.rotation.y, base2moonyard_transform.transform.rotation.z, base2moonyard_transform.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // RCLCPP_INFO(this->get_logger(), "roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);

    Eigen::Affine3f afine_transform = Eigen::Affine3f::Identity();
    afine_transform.translation() << base2moonyard_transform.transform.translation.x, base2moonyard_transform.transform.translation.y, base2moonyard_transform.transform.translation.z;
    afine_transform.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    afine_transform.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
    afine_transform.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud(*cloud, *transformed_cloud, afine_transform);

    return transformed_cloud;
}

void WorldModel::fuseMap(const sensor_msgs::msg::PointCloud2::SharedPtr msg)  {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_local_map(new pcl::PointCloud<pcl::PointXYZ>);
    try
    {
        cropped_cloud_local_map = transformMap(msg);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return;
    }

    std::vector<double> elevation_values(global_map_.info.width*global_map_.info.height, 0.0);
    std::vector<double> density_values(global_map_.info.width*global_map_.info.height, 0.0);

    for(size_t i = 0; i < cropped_cloud_local_map->points.size(); i++){
        int col_x =  int(cropped_cloud_local_map->points[i].x / global_map_.info.resolution ) + global_map_.info.width/2;
        int row_y =  int(cropped_cloud_local_map->points[i].y / global_map_.info.resolution ) + global_map_.info.height/2;

        col_x = std::min(std::max(col_x, 0), int(global_map_.info.width-1));
        row_y = std::min(std::max(row_y, 0), int(global_map_.info.height-1));

        int global_idx = col_x + row_y*global_map_.info.width;
        double elev = cropped_cloud_local_map->points[i].z;
        elevation_values[global_idx] += (elev+0.5)/MAP_RESOLUTION;
        density_values[global_idx] += 1.0;
    }

    for(size_t i = 0; i < global_map_.info.width*global_map_.info.height; i++){
        if(density_values[i] > 1.0){
            global_map_.data[i] = int(elevation_values[i]/density_values[i]);
            filtered_global_map_.data[i] = int(elevation_values[i]/density_values[i]);
        }
    }
    

    // buildWorldModel();
    RCLCPP_INFO(this->get_logger(), "World Model built");
    filterMap();
    // display that map is built but keep updating in the display

    global_map_publisher_->publish(global_map_);
}


void WorldModel::buildWorldModel(){

    for(size_t i = 0; i < elevation_costmap_.data.size(); i++){
        if (global_map_.data[i] != 0 && global_map_.data[i] < 20){
            elevation_costmap_.data[i] = 100 - global_map_.data[i];
        }        
        else{
            elevation_costmap_.data[i] = 0;
        }
    }
    // int neighbour_deltas[8] = [-1, 1, -global_map_.info.width, global_map_.info.width, -global_map_.info.width-1, -global_map_.info.width+1, global_map_.info.width-1, global_map_.info.width+1];
    int neighbour_deltas[8] = {-1, 1, -(int)global_map_.info.width, (int)global_map_.info.width, -(int)global_map_.info.width-1, -(int)global_map_.info.width+1, (int)global_map_.info.width-1, (int)global_map_.info.width+1};

    for(size_t i=0; i<slope_costmap_.data.size(); i++){
        double max_neighbour = -100, min_neighbour = 100;
        for(size_t j=0;j<8;j++){
            size_t neighbour_idx = i+neighbour_deltas[j];
            if(neighbour_idx > slope_costmap_.data.size()){
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
            // RCLCPP_INFO(this->get_logger(), "max: %f, min: %f, slope: %f", max_neighbour, min_neighbour, 10*(max_neighbour - min_neighbour));
        }
    }
    
    for(size_t i=0;i<world_model_.data.size();i++){
        world_model_.data[i] = std::max(elevation_costmap_.data[i], slope_costmap_.data[i]);
    }
    world_model_publisher_->publish(world_model_);

}


void WorldModel::filterMap(){
    double gradient = 1/0.866;
    // use globalmap to update bayes filter and then update filtered global map
    for(size_t i = 0; i < global_map_.info.width*global_map_.info.height; i++){
        if(global_map_.data[i] == 0){
            continue;
        }
        // RCLCPP_INFO(this->get_logger(), "Bayes Filter initialized5");
        if(abs(filtered_global_map_.data[i] - global_map_.data[i]) > gradient){
            bayes_filter_[i].updateCell(global_map_.data[i], 10.0);
            filtered_global_map_.data[i] = int(bayes_filter_[i].getCellElevation());
        }

        // double 
        // update cell of neighbours
        int neighbour_deltas[8] = {-1, 1, -(int)global_map_.info.width, (int)global_map_.info.width, -(int)global_map_.info.width-1, -(int)global_map_.info.width+1, (int)global_map_.info.width-1, (int)global_map_.info.width+1};
        // only 4 neighbours
        // int neighbour_deltas[4] = {-1, 1, -global_map_.info.width, global_map_.info.width};
        for(size_t j=0;j<4;j++){
            size_t neighbour_idx = i+neighbour_deltas[j];
            if(neighbour_idx > global_map_.info.width*global_map_.info.height){
                continue;
            }
            // else if(filtered_global_map_.data[neighbour_idx] == 0){
            //     continue;
            // }   
            if(abs(filtered_global_map_.data[i] - filtered_global_map_.data[neighbour_idx]) <= gradient){
                continue;
            }
            // else if(global_map_.data[i] > global_map_.data[neighbour_idx]){
            else if(filtered_global_map_.data[i] > filtered_global_map_.data[neighbour_idx]){
                bayes_filter_[neighbour_idx].updateCell(filtered_global_map_.data[i] - gradient, 10000.0);
                filtered_global_map_.data[neighbour_idx] = int(bayes_filter_[neighbour_idx].getCellElevation());
            }
            // // // else if(global_map_.data[i] < global_map_.data[neighbour_idx]){
            else if(filtered_global_map_.data[i] < filtered_global_map_.data[neighbour_idx]){
                bayes_filter_[neighbour_idx].updateCell(filtered_global_map_.data[i] + gradient, 10000.0);
                filtered_global_map_.data[neighbour_idx] = int(bayes_filter_[neighbour_idx].getCellElevation());
            }
        }
    }
    // publish filtered global map
    filtered_global_map_publisher_->publish(filtered_global_map_);
}