/* Author: 
 * Subscribers:
 *    - /topic: description
 * Publishers:
 *    - /topic: description
 * Services:
 *    - /name (type): description
 *
 * - Summary
 * 
 * TODO
 * - Add todos
 * */

#include "lx_planning/task_planner.hpp"

TaskPlanner::TaskPlanner(): Node("task_planner_node"){
    
    // Set up subscriptions, publishers, servers & clients
    setupCommunications();

    RCLCPP_INFO(this->get_logger(), "Task Planner initialized");
}

void TaskPlanner::setupCommunications(){
    // Subscribers

    // Publishers

    // Clients

    // Servers
}