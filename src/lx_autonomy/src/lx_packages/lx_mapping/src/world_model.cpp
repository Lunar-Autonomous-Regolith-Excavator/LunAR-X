/* Author: Anish Senathi
 * Publishers:
 *    - /mapping/global_map: [NavMsgs/OccupancyGrid] Global map
 *    - /mapping/world_model: [NavMsgs/OccupancyGrid] World model
 *    - /mapping/filtered_global_map: [NavMsgs/OccupancyGrid] Filtered global map
 *    - /map: [NavMsgs/OccupancyGrid] Traversibility costmap
 * Services:
 *    - /mapping/map_switch: [LxMsgs/Switch] Switch to turn on/off mapping
 *    - /world_model/requested_points: [LxMsgs/RequestRoverService] Service to update the world model
 *
 * - Summary
 * - Generates the elevation map from the point cloud 
 * - Fuses the elevation map with the global map using a bayes filter
 * - Generates the world model from the global map considering the restricted zones and berm zones
 * */


#include "lx_mapping/world_model.hpp"

#define GETMAXINDEX(x, y, width) ((y) * (width) + (x))

WorldModel::WorldModel() : Node("world_model_node")
{   
    debug_mode_ = false;

    node_state_ = false;

    // Setup Communications
    setupCommunications();

    // Setup Maps
    configureMaps();

    // Initializing occupancy grid
    this->global_map_.data.resize(global_map_.info.width*global_map_.info.height);
    this->filtered_global_map_.data.resize(global_map_.info.width*global_map_.info.height);
    this->elevation_costmap_.data.resize(global_map_.info.width*global_map_.info.height);
    this->world_model_.data.resize(global_map_.info.width*global_map_.info.height);
    this->zone_costmap_.data.resize(zone_costmap_.info.width*zone_costmap_.info.height);
    this->berm_costmap_.data.resize(berm_costmap_.info.width*berm_costmap_.info.height);
    this->traversibility_costmap_.data.resize(traversibility_costmap_.info.width*traversibility_costmap_.info.height);
    for(size_t i = 0; i < this->global_map_.info.width*this->global_map_.info.height; i++){
        this->global_map_.data[i] = 0;
        this->filtered_global_map_.data[i] = 0;
    }

    RCLCPP_INFO(this->get_logger(), "World Model initialized");

    setupInitialMaps();
}

void WorldModel::setupInitialMaps(){
    for (size_t j = zone_costmap_.info.height - 1; j > zone_costmap_.info.height - 1 - 20; j--) {
      for (size_t i = zone_costmap_.info.width - 1; i > zone_costmap_.info.width - 1 - 20; i--) {
        zone_costmap_.data[GETMAXINDEX(i, j, zone_costmap_.info.width)] = 100;
      }
    }

    // Make the borders of the map occupied
    for (size_t i = 0; i < zone_costmap_.info.width; i++) {
      zone_costmap_.data[GETMAXINDEX(i, 0, zone_costmap_.info.width)] = 100;
      zone_costmap_.data[GETMAXINDEX(i, zone_costmap_.info.height - 1, zone_costmap_.info.width)] = 100;
    }
    for (size_t i = 0; i < zone_costmap_.info.height; i++) {
      zone_costmap_.data[GETMAXINDEX(0, i, zone_costmap_.info.width)] = 100;
      zone_costmap_.data[GETMAXINDEX(zone_costmap_.info.width - 1, i, zone_costmap_.info.width)] = 100;
    }

    updateTraversibilityCostmapWorldModel();
}

void WorldModel::setupCommunications(){
    rclcpp::QoS qos(10);  // initialize to default QoS
    qos.transient_local();
    qos.reliable(); 
    qos.keep_last(1);

    // Publishers
    global_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("mapping/global_map", 10);
    world_model_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("mapping/world_model", 10);
    filtered_global_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("mapping/filtered_global_map", 10);
    traversibility_costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", qos);

    // Servers
    map_switch_server_ = this->create_service<lx_msgs::srv::Switch>("mapping/map_switch", 
                                                            std::bind(&WorldModel::mapSwitchCallback, 
                                                            this, std::placeholders::_1, std::placeholders::_2));
    world_model_points_server_ = this->create_service<lx_msgs::srv::RequestRoverService>("world_model/requested_points",
                                                            std::bind(&WorldModel::requestedPointsCallback, 
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

    // Set map properties
    zone_costmap_.header.frame_id = "map";
    zone_costmap_.info.map_load_time = this->get_clock()->now();
    zone_costmap_.info.resolution = 0.05;  // replace with your map resolution
    zone_costmap_.info.width = 135;       // replace with your map width
    zone_costmap_.info.height = 135;      // replace with your map height
    zone_costmap_.info.origin.position.x = 0.1;  // replace with your map origin x
    zone_costmap_.info.origin.position.y = 0.1;  // replace with your map origin y
    // zone_costmap_.info.origin.position.x = -13;  // replace with your map origin x
    // zone_costmap_.info.origin.position.y = -9;  // replace with your map origin y
    zone_costmap_.info.origin.position.z = 0.0;
    zone_costmap_.info.origin.orientation.x = 0.0;
    zone_costmap_.info.origin.orientation.y = 0.0;
    zone_costmap_.info.origin.orientation.z = 0.0;
    zone_costmap_.info.origin.orientation.w = 1.0;

    berm_costmap_.header.frame_id = "map";
    berm_costmap_.info = zone_costmap_.info;

    world_model_.header.frame_id = "map";
    world_model_.info = global_map_.info;

    traversibility_costmap_.header.frame_id = "map";
    traversibility_costmap_.info = zone_costmap_.info;

    // Initialize bayes filter
    for(size_t i = 0; i < global_map_.info.width*global_map_.info.height; i++){
        BayesFilter bf;
        bayes_filter_.push_back(bf);
    }
}


void WorldModel::mapSwitchCallback(const std::shared_ptr<lx_msgs::srv::Switch::Request> req,
                                                std::shared_ptr<lx_msgs::srv::Switch::Response> res){
    if(req->switch_state && node_state_ == false){
        RCLCPP_INFO(this->get_logger(), "Elevation mapping switched ON");
        // Subscribe to the Point cloud topic
        transformed_pcl_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("mapping/transformed_pointcloud", 10, 
                                                                                    std::bind(&WorldModel::transformedPCLCallback, this, std::placeholders::_1));

        timer_global_map_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&WorldModel::publishGlobalMap, this));
        node_state_ = true;
    }
    else if (req->switch_state == false && node_state_ == true){
        RCLCPP_INFO(this->get_logger(), "Elevation mapping switched OFF");
        transformed_pcl_subscriber_.reset();
        node_state_ = false;
    }
    res->success = true;
}

void WorldModel::requestedPointsCallback(const std::shared_ptr<lx_msgs::srv::RequestRoverService::Request> req,
                                                std::shared_ptr<lx_msgs::srv::RequestRoverService::Response> res){
    update_traversibility_thread_ = std::thread(std::bind(&WorldModel::updateTraversibilityCostmapCallback, this, req, res));

    update_traversibility_thread_.detach();
}

void WorldModel::updateTraversibilityCostmapCallback(const std::shared_ptr<lx_msgs::srv::RequestRoverService::Request> req,
                                                std::shared_ptr<lx_msgs::srv::RequestRoverService::Response> res){
                                                
    buildRestrictedZonesWorldModel(req->restricted_zone_coordinates);
    updateBermZonesWorldModel(req->berm.berm_configuration);
    updateTraversibilityCostmapWorldModel();

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
        elevation_values[global_idx] += ELEVATION_SCALE*(elev-0.05);
        density_values[global_idx] += 1.0;
    }

    for(size_t i = 0; i < global_map_.info.width*global_map_.info.height; i++){
        if(density_values[i] > 1.0){
            global_map_.data[i] = int(elevation_values[i]/density_values[i]);
            filtered_global_map_.data[i] = int(elevation_values[i]/density_values[i]);
        }
    }
    
    filterMap();
}

bool WorldModel::isPointInsidePolygon(geometry_msgs::msg::Point32& point, std::vector<geometry_msgs::msg::PointStamped> polygon){
    bool inside = false;
    size_t i, j = polygon.size() - 1;

    if(polygon.size() < 3){
        return false;
    }

    for (i = 0; i < polygon.size(); i++) {
        if (((polygon[i].point.y > point.y) != (polygon[j].point.y > point.y)) &&
            (point.x < (polygon[j].point.x - polygon[i].point.x) * (point.y - polygon[i].point.y) /
                       (polygon[j].point.y - polygon[i].point.y) + polygon[i].point.x)) {
            inside = !inside;
        }
        j = i;
    }

    return inside;
}

void WorldModel::buildRestrictedZonesWorldModel(std::vector<geometry_msgs::msg::PointStamped> restricted_zone_coordinates){

    for(size_t i = 0; i < zone_costmap_.info.width*zone_costmap_.info.height; i++){
        geometry_msgs::msg::Point32 point;
        point.x = zone_costmap_.info.origin.position.x + (i%zone_costmap_.info.width)*zone_costmap_.info.resolution;
        point.y = zone_costmap_.info.origin.position.y + (i/zone_costmap_.info.width)*zone_costmap_.info.resolution;
        point.z = 0.0;
        if(isPointInsidePolygon(point, restricted_zone_coordinates)){
            zone_costmap_.data[i] = 100;
        }
    }
}

void WorldModel::updateTraversibilityCostmapWorldModel(){

    for(size_t i = 0; i < traversibility_costmap_.data.size(); i++){
        traversibility_costmap_.data[i] = std::max(berm_costmap_.data[i], zone_costmap_.data[i]);
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

void WorldModel::updateBermZonesWorldModel(std::vector<geometry_msgs::msg::PointStamped> berm_zone_coordinates){

    double x_first = berm_zone_coordinates[0].point.x;
    double y_first = berm_zone_coordinates[0].point.y;
    double x_last = berm_zone_coordinates.back().point.x;
    double y_last = berm_zone_coordinates.back().point.y;

    // berm_zone_coordinates is a vector of points that define the berm line segment. it is not a polygon. set berm_zone to 100 if point is within 0.3m of berm line
    for(size_t i = 0; i < berm_costmap_.info.width*berm_costmap_.info.height; i++){
        geometry_msgs::msg::Point32 point;
        point.x = berm_costmap_.info.origin.position.x + (i%berm_costmap_.info.width)*berm_costmap_.info.resolution;
        point.y = berm_costmap_.info.origin.position.y + (i/berm_costmap_.info.width)*berm_costmap_.info.resolution;
        point.z = 0.0;
        for(size_t j = 1; j < berm_zone_coordinates.size()-1; j++){
            double x1 = berm_zone_coordinates[j].point.x;
            double y1 = berm_zone_coordinates[j].point.y;
            double x2 = berm_zone_coordinates[j+1].point.x;
            double y2 = berm_zone_coordinates[j+1].point.y;
            double line_dist = abs((y2-y1)*point.x - (x2-x1)*point.y + x2*y1 - y2*x1)/sqrt(pow(y2-y1, 2) + pow(x2-x1, 2));
            double point_dist = sqrt(pow(point.x - x1, 2) + pow(point.y - y1, 2));
            if(line_dist < (GLOBAL_BERM_LENGTH_M/2) && point_dist < (GLOBAL_BERM_LENGTH_M*1.2)){
                berm_costmap_.data[i] = 100;
                break;
            }
        }
        double dist_first = sqrt(pow(point.x - x_first, 2) + pow(point.y - y_first, 2));
        double dist_last = sqrt(pow(point.x - x_last, 2) + pow(point.y - y_last, 2));
        if(dist_first < GLOBAL_BERM_LENGTH_M/2 || dist_last < GLOBAL_BERM_LENGTH_M/2){
            berm_costmap_.data[i] = 100;
        }        
    }
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