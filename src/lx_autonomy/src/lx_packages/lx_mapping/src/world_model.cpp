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
    this->elevation_costmap_.data.resize(global_map_.info.width*global_map_.info.height);
    this->world_model_.data.resize(global_map_.info.width*global_map_.info.height);
    this->zone_costmap_.data.resize(global_map_.info.width*global_map_.info.height);
    this->traversibility_costmap_.data.resize(global_map_.info.width*global_map_.info.height);
    for(size_t i = 0; i < this->global_map_.info.width*this->global_map_.info.height; i++){
        this->global_map_.data[i] = 0;
        this->filtered_global_map_.data[i] = 0;
    }

    RCLCPP_INFO(this->get_logger(), "World Model initialized");

    buildRestrictedZonesWorldModel();


}

void WorldModel::setupCommunications(){

    // Publishers
    global_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("mapping/global_map2", 10);
    world_model_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("mapping/world_model", 10);
    filtered_global_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("mapping/filtered_global_map2", 10);
    traversibility_costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("mapping/traversibility_costmap", 10);

    // Servers
    map_switch_server_ = this->create_service<lx_msgs::srv::Switch>("mapping/map_switch", 
                                                            std::bind(&WorldModel::mapSwitchCallback, 
                                                            this, std::placeholders::_1, std::placeholders::_2));

    // Transform Listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_traversibility_costmap_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&WorldModel::publishTraversibilityCostmap, this));
}

void WorldModel::configureMaps(){
    // Configuring occupancy grid

    this->global_map_.header.frame_id = "map";
    this->global_map_.info.resolution = MAP_RESOLUTION;
    this->global_map_.info.width = MAP_DIMENSION/MAP_RESOLUTION;
    this->global_map_.info.height = MAP_DIMENSION/MAP_RESOLUTION;
    this->global_map_.info.origin.position.x = 0.0;
    this->global_map_.info.origin.position.y = 0.0;

    filtered_global_map_.header.frame_id = "map";
    filtered_global_map_.info = global_map_.info;

    elevation_costmap_.header.frame_id = "map";
    elevation_costmap_.info = global_map_.info;

    berm_costmap_.header.frame_id = "map";
    berm_costmap_.info = global_map_.info;

    zone_costmap_.header.frame_id = "map";
    zone_costmap_.info = global_map_.info;

    world_model_.header.frame_id = "map";
    world_model_.info = global_map_.info;

    traversibility_costmap_.header.frame_id = "map";
    traversibility_costmap_.info = global_map_.info;

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

        timer_global_map_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&WorldModel::publishGlobalMap, this));
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
    
    geometry_msgs::msg::TransformStamped base2map_transform;
    try
    {
      base2map_transform = tf_buffer_->lookupTransform("map","base_link",tf2::TimePointZero, tf2::durationFromSec(10));
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", ex.what());

        return nullptr;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // convert x,y,z,w to roll, pitch, yaw
    tf2::Quaternion q(base2map_transform.transform.rotation.x, base2map_transform.transform.rotation.y, base2map_transform.transform.rotation.z, base2map_transform.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // RCLCPP_INFO(this->get_logger(), "roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);

    Eigen::Affine3f afine_transform = Eigen::Affine3f::Identity();
    afine_transform.translation() << base2map_transform.transform.translation.x, base2map_transform.transform.translation.y, base2map_transform.transform.translation.z;
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
        int col_x =  int(cropped_cloud_local_map->points[i].x / global_map_.info.resolution );
        int row_y =  int(cropped_cloud_local_map->points[i].y / global_map_.info.resolution );

        col_x = std::min(std::max(col_x, 0), int(global_map_.info.width-1));
        row_y = std::min(std::max(row_y, 0), int(global_map_.info.height-1));

        int global_idx = col_x + row_y*global_map_.info.width;
        double elev = cropped_cloud_local_map->points[i].z;
        elevation_values[global_idx] += ELEVATION_SCALE*(elev-0.1);
        density_values[global_idx] += 1.0;
    }

    for(size_t i = 0; i < global_map_.info.width*global_map_.info.height; i++){
        if(density_values[i] > 1.0){
            global_map_.data[i] = int(elevation_values[i]/density_values[i]);
            filtered_global_map_.data[i] = int(elevation_values[i]/density_values[i]);
        }
    }
    
    filterMap();
    updateElevationWorldModel();
}


void WorldModel::buildRestrictedZonesWorldModel(){
    geometry_msgs::msg::PointStamped ts_zone[4], speaker_zone[4], safety_person_zone[4];

    // ts_zone[0].header.frame_id = "map";
    // ts_zone[1].header.frame_id = "map";
    // ts_zone[2].header.frame_id = "map";
    // ts_zone[3].header.frame_id = "map";
    // ts_zone[0].point.x = 0.0; ts_zone[0].point.y = 0.0; ts_zone[0].point.z = 0.0;
    // ts_zone[1].point.x = 1.0; ts_zone[1].point.y = 0.0; ts_zone[1].point.z = 0.0;
    // ts_zone[2].point.x = 1.0; ts_zone[2].point.y = 1.0; ts_zone[2].point.z = 0.0;
    // ts_zone[3].point.x = 0.0; ts_zone[3].point.y = 1.0; ts_zone[3].point.z = 0.0;

    for(size_t i = 0; i < 4; i++){
        ts_zone[i].header.frame_id = "map";
        speaker_zone[i].header.frame_id = "map";
        safety_person_zone[i].header.frame_id = "map";
    }

    auto neighbour_points = [](geometry_msgs::msg::PointStamped* points, double x, double y, double z){
        points[0].point.x = x; points[0].point.y = y; points[0].point.z = z;
        points[1].point.x = x+1.0; points[1].point.y = y; points[1].point.z = z;
        points[2].point.x = x+1.0; points[2].point.y = y+1.0; points[2].point.z = z;
        points[3].point.x = x; points[3].point.y = y+1.0; points[3].point.z = z;
    };

    neighbour_points(ts_zone, 0.0, 0.0, 0.0);
    neighbour_points(speaker_zone, 4.0, 0.0, 0.0);
    neighbour_points(safety_person_zone, 7.0, 4.0, 0.0);

    // for all points inside ts_zone, set zone_costmap_.data[i] = 100
    for(size_t i = 0; i < zone_costmap_.data.size(); i++){
        if(zone_costmap_.data[i] == 100){
            continue;
        }
        // convert i to x,y
        int col_x = i % zone_costmap_.info.width;
        int row_y = i / zone_costmap_.info.width;
        // convert x,y to world coordinates
        double x = col_x*zone_costmap_.info.resolution + zone_costmap_.info.origin.position.x;
        double y = row_y*zone_costmap_.info.resolution + zone_costmap_.info.origin.position.y;
        // check if point is inside ts_zone
        if(x >= ts_zone[0].point.x && x <= ts_zone[1].point.x && y >= ts_zone[0].point.y && y <= ts_zone[3].point.y){
            zone_costmap_.data[i] = 100;
        }
        else if(x >= speaker_zone[0].point.x && x <= speaker_zone[1].point.x && y >= speaker_zone[0].point.y && y <= speaker_zone[3].point.y){
            zone_costmap_.data[i] = 100;
        }
        else if(x >= safety_person_zone[0].point.x && x <= safety_person_zone[1].point.x && y >= safety_person_zone[0].point.y && y <= safety_person_zone[3].point.y){
            zone_costmap_.data[i] = 100;
        }
    }

    updateTraversibilityCostmapWorldModel();
}

void WorldModel::updateTraversibilityCostmapWorldModel(){

    //traversibility_costmap_ is same as zone_costmap_ for now
    for(size_t i = 0; i < traversibility_costmap_.data.size(); i++){
        traversibility_costmap_.data[i] = zone_costmap_.data[i];
    }
    RCLCPP_INFO(this->get_logger(), "Traversibility Costmap updated");
}

void WorldModel::publishTraversibilityCostmap(){
    traversibility_costmap_publisher_->publish(traversibility_costmap_);
}

void WorldModel::publishGlobalMap(){
    global_map_publisher_->publish(global_map_);
    filtered_global_map_publisher_->publish(filtered_global_map_);
    // world_model_publisher_->publish(world_model_);
}

void WorldModel::updateBermZonesWorldModel(){

}

void WorldModel::updateElevationWorldModel(){

    for(size_t i = 0; i < global_map_.data.size(); i++){
        if (global_map_.data[i] != 0 && global_map_.data[i] < 20){
            elevation_costmap_.data[i] = 100 - global_map_.data[i];
        }        
        else{
            elevation_costmap_.data[i] = 0;
        }
        
    }
    RCLCPP_INFO(this->get_logger(), "Elevation Costmap initialized");
    // int neighbour_deltas[8] = [-1, 1, -global_map_.info.width, global_map_.info.width, -global_map_.info.width-1, -global_map_.info.width+1, global_map_.info.width-1, global_map_.info.width+1];
    int neighbour_deltas[8] = {-1, 1, -(int)global_map_.info.width, (int)global_map_.info.width, -(int)global_map_.info.width-1, -(int)global_map_.info.width+1, (int)global_map_.info.width-1, (int)global_map_.info.width+1};

    // for(size_t i=0; i<slope_costmap_.data.size(); i++){
    //     double max_neighbour = -100, min_neighbour = 100;
    //     for(size_t j=0;j<8;j++){
    //         size_t neighbour_idx = i+neighbour_deltas[j];
    //         if(neighbour_idx > slope_costmap_.data.size()){
    //             continue;
    //         }
    //         else if(global_map_.data[neighbour_idx] == 0){
    //             continue;
    //         }   
    //         if(global_map_.data[i+neighbour_deltas[j]] > max_neighbour){
    //             max_neighbour = global_map_.data[i+neighbour_deltas[j]];
    //         }
    //         if(global_map_.data[i+neighbour_deltas[j]] < min_neighbour){
    //             min_neighbour = global_map_.data[i+neighbour_deltas[j]];
    //         }
    //     }
    //     if(max_neighbour == -100 || min_neighbour == 100){
    //         slope_costmap_.data[i] = 0;
    //     }
    //     else{
    //         slope_costmap_.data[i] = max_neighbour - min_neighbour;
    //     }
    // }
    
    // for(size_t i=0;i<world_model_.data.size();i++){
    //     world_model_.data[i] = std::max(elevation_costmap_.data[i], slope_costmap_.data[i]);
    // }
    // RCLCPP_INFO(this->get_logger(), "Slope Costmap initialized");
    // world_model_publisher_->publish(elevation_costmap_);
}


void WorldModel::filterMap(){
    double gradient = ELEVATION_SCALE/0.866;
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
}