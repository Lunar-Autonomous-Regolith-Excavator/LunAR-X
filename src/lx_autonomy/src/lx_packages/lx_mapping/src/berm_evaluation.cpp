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
    berm_evaluation_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mapping/berm_evaluation_array", 10);
    peak_points_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("mapping/berm_peak_points", 10);
    // Servers
    berm_points_server_ = this->create_service<lx_msgs::srv::BermService>("berm_evaluation/requested_berm_points", 
                                    std::bind(&BermEvaluation::userBermPointsCB, this, std::placeholders::_1, std::placeholders::_2));

    berm_eval_server_ = this->create_service<lx_msgs::srv::BermProgressEval>("berm_evaluation/berm_progress", 
                                    std::bind(&BermEvaluation::evalServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
}


void BermEvaluation::userBermPointsCB(const std::shared_ptr<lx_msgs::srv::BermService::Request> req,
                                          const std::shared_ptr<lx_msgs::srv::BermService::Response> res){
    requested_berm_points_.clear();
    
    // Store requested berm points for evaluation
    for(auto &point: req->berm.berm_configuration){
        requested_berm_points_.push_back(point);
        berm_progress_.berm_points.push_back(point);
    }

    RCLCPP_INFO(this->get_logger(), "Received berm goal points for berm evaluation");

    res->success = true;
}


void BermEvaluation::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    map_update_thread_ = std::thread(std::bind(&BermEvaluation::saveUpdatedMap, this, msg));

    map_update_thread_.detach();
}

void BermEvaluation::evalServiceCallback(const std::shared_ptr<lx_msgs::srv::BermProgressEval::Request> req,
                                          const std::shared_ptr<lx_msgs::srv::BermProgressEval::Response> res){

    if(!req->need_metrics){
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Evaluating berm");

    // print the requested berm points
    for(size_t i = 0; i < requested_berm_points_.size(); i++){
        RCLCPP_INFO(this->get_logger(), "Requested berm point %d: %f, %f", (int)i, requested_berm_points_[i].point.x, requested_berm_points_[i].point.y);
    }

    // publish the berm evaluation array. every marker is a cuboid of 0.2m x 0.2m x berms_heights[i]*0.15m
    visualization_msgs::msg::MarkerArray marker_array_msg;
    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header.frame_id = "map";
    marker_msg.header.stamp = this->now();
    marker_msg.ns = "berm";
    marker_msg.type = visualization_msgs::msg::Marker::CUBE;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.scale.x = 0.1;
    marker_msg.scale.y = 0.4;
    marker_msg.color.a = 1.0;
    marker_msg.color.r = 0.0; 
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;

    visualization_msgs::msg::Marker peak_points_marker;
    peak_points_marker.header.frame_id = "map";
    peak_points_marker.header.stamp = this->now();
    peak_points_marker.ns = "berm";
    peak_points_marker.type = visualization_msgs::msg::Marker::POINTS;
    peak_points_marker.action = visualization_msgs::msg::Marker::ADD;
    peak_points_marker.scale.x = 0.05;
    peak_points_marker.scale.y = 0.05;
    peak_points_marker.scale.z = 0.05;
    peak_points_marker.color.a = 1.0;
    peak_points_marker.color.r = 0.0;
    peak_points_marker.color.g = 0.0;   
    peak_points_marker.color.b = 1.0;

    int num_bins_per_section = 0.4 / (this->map_->info.resolution * 1.414);
    RCLCPP_INFO(this->get_logger(), "Num bins per section: %d", num_bins_per_section);

    std::vector<double> berm_heights_bins(requested_berm_points_.size()*num_bins_per_section, 0);
    std::vector<geometry_msgs::msg::Point> peak_points(requested_berm_points_.size()*num_bins_per_section, geometry_msgs::msg::Point());

    double peakline_length = 0;
    double peakline_error = 0;

    for(size_t i = 0; i < requested_berm_points_.size()-1; i++){

        geometry_msgs::msg::Point berm_marker_1_point , berm_marker_2_point;
        berm_marker_1_point.x = requested_berm_points_[i].point.x;
        berm_marker_1_point.y = requested_berm_points_[i].point.y;
        berm_marker_1_point.z = 0;
        berm_marker_2_point.x = requested_berm_points_[i+1].point.x;
        berm_marker_2_point.y = requested_berm_points_[i+1].point.y;
        berm_marker_2_point.z = 0;

        // break the line segment joining the two points into parts of length 0.4
        double x_diff = berm_marker_2_point.x - berm_marker_1_point.x;
        double y_diff = berm_marker_2_point.y - berm_marker_1_point.y;

        geometry_msgs::msg::Point midpoint;
        midpoint.x = (berm_marker_1_point.x + berm_marker_2_point.x)/2;
        midpoint.y = (berm_marker_1_point.y + berm_marker_2_point.y)/2;
        midpoint.z = 0;

        //find the mean elevation of the region 0.3 to 0.4 m away from the line joining the two points
        double sum_ground_height = 0;
        int num_points_in_region = 1;
        // loop over the the rectangle region parallel to the line joining the two berm_marker_points 1 and 2 but 0.3 to 0.4 m away from it
        double m = y_diff/(x_diff+0.015);
        double c = berm_marker_1_point.y - m*berm_marker_1_point.x;
        for(size_t j = 0; j < this->map_->data.size(); j++){
            double x = (j%this->map_->info.width)*this->map_->info.resolution;
            double y = (j/this->map_->info.width)*this->map_->info.resolution;
            double line_dist = abs(m*x - y + c)/sqrt(pow(m, 2) + 1);
            if(line_dist > 0.4 && line_dist < 0.5 && this->map_->data[j] > 0){
                sum_ground_height += this->map_->data[j];
                num_points_in_region++;
            }
        }

        double mean_ground_height = sum_ground_height/num_points_in_region;

        RCLCPP_INFO(this->get_logger(), "Mean ground height: %f", mean_ground_height);

        double max = -100;
        for(size_t j = 0; j < this->map_->data.size(); j++){
            double x = (j%this->map_->info.width)*this->map_->info.resolution;
            double y = int(j/this->map_->info.width)*this->map_->info.resolution;
            double dist = sqrt(pow(x - midpoint.x, 2) + pow(y - midpoint.y, 2));
            if(dist < 0.3){
                if(this->map_->data[j] > max){
                    max = this->map_->data[j];
                }

                double x_proj, y_proj, parallel_dist, perpendicular_dist, bin;

                if(abs(m) > 20){
                    parallel_dist = y - berm_marker_1_point.y;
                    perpendicular_dist = x - berm_marker_1_point.x;
                    bin = int(parallel_dist/(this->map_->info.resolution * 1.414));
                }
                
                else{                
                    x_proj = (m*(y - berm_marker_1_point.y) + x + m*m*berm_marker_1_point.x)/(m*m + 1);
                    y_proj = m*x_proj + c;      
                    int sign = (y_diff > 0) - (y_diff < 0);
                    parallel_dist = (y_proj - berm_marker_1_point.y)*m/(m*m + 1)*sign;
                    perpendicular_dist = (m*x - y + c)/sqrt(pow(m, 2) + 1);
                    bin = int(parallel_dist/(this->map_->info.resolution * 1.414));
                }

                if(bin < num_bins_per_section && bin >= 0){
                    if(this->map_->data[j] > berm_heights_bins[i*num_bins_per_section + bin]){
                        berm_heights_bins[i*num_bins_per_section + bin] = this->map_->data[j];
                        peakline_error += pow(perpendicular_dist, 2);
                        geometry_msgs::msg::Point point;
                        point.x = x; point.y = y; point.z = this->map_->data[j]/ELEVATION_SCALE;
                        peak_points[i*num_bins_per_section + bin] = point;
                    }
                }
            }
        }

        auto berm_height = (max - mean_ground_height)/ELEVATION_SCALE;
        RCLCPP_INFO(this->get_logger(), "Berm height in section %d: %f", (int)i, berm_height);

        marker_msg.id = i;
        marker_msg.pose.position.x = midpoint.x;
        marker_msg.pose.position.y = midpoint.y;
        marker_msg.pose.position.z = berm_height*0.5;
        marker_msg.scale.z = berm_height;
        if(i%2==0){
            marker_msg.color.b = 0.0;
        }
        else{
            marker_msg.color.b = 0.13;
        }
        marker_msg.colors.push_back(marker_msg.color);
        marker_array_msg.markers.push_back(marker_msg);

        berm_progress_.heights.push_back(berm_height);
    }
    
    for(size_t i = 0; i < berm_heights_bins.size(); i++){
        if(berm_heights_bins[i] >= 0.9*DESIRED_BERM_HEIGHT_M){
            peakline_length += this->map_->info.resolution * 1.414;
        }
    }
    peakline_error = sqrt(peakline_error)*0.015/(peakline_length+0.0001);

    berm_progress_.length = peakline_length;
    berm_progress_.peakline_error = peakline_error;

    RCLCPP_INFO(this->get_logger(), "Peakline length: %f, Peakline error: %f", peakline_length, peakline_error);
    

    for(size_t i = 0; i < peak_points.size(); i++){
        peak_points_marker.points.push_back(peak_points[i]);
    }

    berm_evaluation_array_publisher_->publish(marker_array_msg);
    peak_points_publisher_->publish(peak_points_marker);

    res->progress = berm_progress_;
}

// TODO: make it a service
void BermEvaluation::saveUpdatedMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    this->map_ = msg;
}
