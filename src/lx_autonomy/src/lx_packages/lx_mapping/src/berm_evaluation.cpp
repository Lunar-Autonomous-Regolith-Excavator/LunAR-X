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
 * - Test with rover
 * - Check compatibility with planner
 * */


#include "lx_mapping/berm_evaluation.hpp"
#include <vector>

BermEvaluation::BermEvaluation() : Node("berm_evaluation_node"){       
    // Clear data storage
    requested_berm_points_.clear();
    
    // Setup Communications
    setupCommunications();

    RCLCPP_INFO(this->get_logger(), "Berm Evaluation initialized");
}


void BermEvaluation::setupCommunications(){
    // Subscribers
    occupancy_grid_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("mapping/global_map", 10, 
                                                                                        std::bind(&BermEvaluation::mapCallback, this, std::placeholders::_1));
    // Publishers
    berm_marker_1_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("mapping/berm_marker_1", 10);
    berm_marker_2_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("mapping/berm_marker_2", 10);
    berm_evaluation_array_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("mapping/berm_evaluation_array", 10);

    // Servers
    berm_points_server_ = this->create_service<lx_msgs::srv::BermService>("berm_evaluation/requested_berm_points", 
                                    std::bind(&BermEvaluation::userBermPointsCB, this, std::placeholders::_1, std::placeholders::_2));
}


void BermEvaluation::userBermPointsCB(const std::shared_ptr<lx_msgs::srv::BermService::Request> req,
                                          const std::shared_ptr<lx_msgs::srv::BermService::Response> res){
    requested_berm_points_.clear();
    
    // Store requested berm points for evaluation
    for(auto &point: req->berm.berm_configuration){
        requested_berm_points_.push_back(point);
    }

    RCLCPP_INFO(this->get_logger(), "Received berm goal points for berm evaluation");

    res->success = true;
}


void BermEvaluation::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    berm_evaluation_thread_ = std::thread(std::bind(&BermEvaluation::bermEval, this, msg));

    berm_evaluation_thread_.detach();
}


void BermEvaluation::bermEval(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){

    // TODO: create berm_markers_1 and 2 for end points of berm and publish them
    berm_marker_1_.header.frame_id = "moonyard";
    berm_marker_1_.header.stamp = this->now();
    berm_marker_1_.ns = "berm";
    berm_marker_1_.id = 0;
    berm_marker_1_.type = visualization_msgs::msg::Marker::SPHERE;
    berm_marker_1_.action = visualization_msgs::msg::Marker::ADD;
    berm_marker_1_.pose.position.x = -2.5;
    berm_marker_1_.pose.position.y = 3;
    berm_marker_1_.pose.position.z = 0;
    berm_marker_1_.scale.x = 0.5;
    berm_marker_1_.scale.y = 0.5;
    berm_marker_1_.scale.z = 0.5;
    berm_marker_1_.color.a = 1.0; // Don't forget to set the alpha!
    berm_marker_1_.color.r = 0.0;
    berm_marker_1_.color.g = 1.0;
    berm_marker_1_.color.b = 0.0;

    berm_marker_2_.header.frame_id = "moonyard";
    berm_marker_2_.header.stamp = this->now();
    berm_marker_2_.ns = "berm";
    berm_marker_2_.id = 0;
    berm_marker_2_.type = visualization_msgs::msg::Marker::SPHERE;
    berm_marker_2_.action = visualization_msgs::msg::Marker::ADD;
    berm_marker_2_.pose.position.x = -1.8;
    berm_marker_2_.pose.position.y = 4;
    berm_marker_2_.pose.position.z = 0;
    berm_marker_2_.scale.x = 0.5;
    berm_marker_2_.scale.y = 0.5;
    berm_marker_2_.scale.z = 0.5;
    berm_marker_2_.color.a = 1.0; // Don't forget to set the alpha!
    berm_marker_2_.color.r = 0.0;
    berm_marker_2_.color.g = 0.0;
    berm_marker_2_.color.b = 1.0;

    berm_marker_1_publisher_->publish(berm_marker_1_);
    berm_marker_2_publisher_->publish(berm_marker_2_);

    geometry_msgs::msg::Point berm_marker_1_point , berm_marker_2_point;
    berm_marker_1_point.x = -2.5;
    berm_marker_1_point.y = 3;
    berm_marker_1_point.z = 0;
    berm_marker_2_point.x = -1.8;
    berm_marker_2_point.y = 4;
    berm_marker_2_point.z = 0;

    // break the line segment joining the two points into parts of length 0.4
    std::vector <geometry_msgs::msg::Point> berm_points;
    double x_diff = berm_marker_2_point.x - berm_marker_1_point.x;
    double y_diff = berm_marker_2_point.y - berm_marker_1_point.y;

    double dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
    int num_points = dist/0.4;
    double x_step = x_diff/num_points;
    double y_step = y_diff/num_points;
    for(int i = 0; i < num_points; i++){
        geometry_msgs::msg::Point point;
        point.x = berm_marker_1_point.x + (i+0.5)*x_step;
        point.y = berm_marker_1_point.y + (i+0.5)*y_step;
        point.z = 0;
        berm_points.push_back(point);
    }

    //find the mean elevation of the region 0.3 to 0.4 m away from the line joining the two points
    double sum_ground_height = 0;
    int num_points_in_region = 0;
    // loop over the the rectangle region parallel to the line joining the two berm_marker_points 1 and 2 but 0.3 to 0.4 m away from it
    double m = y_diff/x_diff;
    double c = berm_marker_1_point.y - m*berm_marker_1_point.x;
    for(int i = 0; i < berm_points.size(); i++){
        for(int j = 0; j < msg->data.size(); j++){
            double x = msg->info.origin.position.x + (j%msg->info.width)*msg->info.resolution;
            double y = msg->info.origin.position.y + (j/msg->info.width)*msg->info.resolution;
            double dist = abs(m*x - y + c)/sqrt(pow(m, 2) + 1);
            if(dist > 0.3 && dist < 0.4 && msg->data[j] > 0){
                sum_ground_height += msg->data[j];
                num_points_in_region++;
            }
        }
    }

    double mean_ground_height = sum_ground_height/num_points_in_region;

    // for every point in the vector, find the max value of the occupancy grid in a 0.2m radius
    std::vector <int> berm_heights;

    for(int i = 0; i < berm_points.size(); i++){
        double max = 0;
        for(int j = 0; j < msg->data.size(); j++){
            double x = msg->info.origin.position.x + (j%msg->info.width)*msg->info.resolution;
            double y = msg->info.origin.position.y + (j/msg->info.width)*msg->info.resolution;
            double dist = sqrt(pow(x - berm_points[i].x, 2) + pow(y - berm_points[i].y, 2));
            if(dist < 0.2){
                if(msg->data[j] > max){
                    max = msg->data[j];
                }
            }
        }
        int percetage_completed = int((max - mean_ground_height)/(2*0.15));
        berm_heights.push_back(percetage_completed);
    }
    // print the heights
    for(int i = 0; i < berm_heights.size(); i++){
        RCLCPP_INFO(this->get_logger(), "Percentage completion at point %d: %d", i, berm_heights[i]);
    }

    // publish the berm evaluation array. every marker is a cuboid of 0.2m x 0.2m x berms_heights[i]*0.15m
    visualization_msgs::msg::Marker marker_array_msg;
    marker_array_msg.header.frame_id = "moonyard";
    marker_array_msg.header.stamp = this->now();
    marker_array_msg.ns = "berm";
    marker_array_msg.id = 0;
    marker_array_msg.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker_array_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_array_msg.scale.x = 0.4;
    marker_array_msg.scale.y = 0.4;
    marker_array_msg.scale.z = 1.5;
    marker_array_msg.color.a = 1.0; // Don't forget to set the alpha!
    marker_array_msg.color.r = 0.0; 
    marker_array_msg.color.g = 1.0;
    marker_array_msg.color.b = 0.0;
    for(int i = 0; i < berm_points.size(); i++){
        geometry_msgs::msg::Point point;
        point.x = berm_points[i].x;
        point.y = berm_points[i].y;
        point.z = berm_heights[i]*0.015-1.0;
        marker_array_msg.points.push_back(point);
        marker_array_msg.colors.push_back(marker_array_msg.color);
    }
    berm_evaluation_array_publisher_->publish(marker_array_msg);
}