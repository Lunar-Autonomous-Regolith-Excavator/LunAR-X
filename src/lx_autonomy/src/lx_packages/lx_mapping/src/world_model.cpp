#include "lx_mapping/world_model.hpp"

WorldModel::WorldModel() : Node("world_model_node")
{   
    // set false for dry runs, set true for commands and to publish debug data
    debug_mode_ = true;

    auto qos = rclcpp::SensorDataQoS();
    
    subscription_global_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "lx_mapping/global_map", 10, std::bind(&WorldModel::topic_callback_global_map, this, _1)); //subscribes to the point cloud topic at 1Hz

    publisher_world_model_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("lx_mapping/world_model", 10);
    
    bool debug_mode_;

    nav_msgs::msg::OccupancyGrid global_map_, elevation_costmap_,slope_costmap_, berm_costmap_, zone_costmap_, world_model_;
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