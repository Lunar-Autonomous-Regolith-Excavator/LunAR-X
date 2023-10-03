/* Author: Dhruv Tyagi
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
 * - Add Visualization
 * - Add check of already on-going operation
 * - Add Feasibility Check
 * - Add Action Request
 * */

#include "lx_external_interface/goal_handler.hpp"

GoalHandler::GoalHandler(): Node("goal_handler_node"){  
    user_requested_berm_points_.clear();

    // Set up subscriptions & publishers
    setupCommunications();

    RCLCPP_INFO(this->get_logger(), "Goal Handler initialized");
}

void GoalHandler::setupCommunications(){
    // Subscribers

    // Publishers

    // Clients

    // Servers
    user_berm_request_server_ = this->create_service<lx_msgs::srv::BermService>("request_rover/build_berm", 
                                    std::bind(&GoalHandler::userBermRequestCB, this, std::placeholders::_1, std::placeholders::_2));
}

void GoalHandler::userBermRequestCB(const std::shared_ptr<lx_msgs::srv::BermService::Request> req,
                                          const std::shared_ptr<lx_msgs::srv::BermService::Response> res){
    user_requested_berm_points_.clear();
    RCLCPP_INFO(this->get_logger(), "Received berm request");

    // Store requested berm points 
    for(auto &point : req->berm.berm_configuration){
        user_requested_berm_points_.push_back(point);
        // Debug print points
        RCLCPP_INFO(this->get_logger(), "Point: %.2f, %.2f", point.point.x, point.point.y);
    }
    
    // Start thread to check for berm feasibility
    feasibility_check_thread_ = std::thread(&GoalHandler::checkBermFeasibility, this);

    // Detach thread
    feasibility_check_thread_.detach();

    // Set response success to indicate goal is received
    res->success = true;
}

void GoalHandler::checkBermFeasibility(){
    // TODO
    RCLCPP_INFO(this->get_logger(), "Checking berm feasibility");

    // Check if berm is feasible

    // If feasible, send berm action request to operations handler

}

void GoalHandler::visualizeFeasibleBerm(){
    // TODO
}
