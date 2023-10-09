/* Author: Hariharan Ravichandran
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
 * - Get tool height pose 
 * - Collision check in two layers
 * */

#include "lx_planning/task_planner.hpp"

// Function to convert euler angles to quaternion
geometry_msgs::msg::Quaternion eulerToQuaternion(double roll, double pitch, double yaw){
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
    quaternion.y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
    quaternion.z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
    quaternion.w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
    return quaternion;
}


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

    // Service servers
    this->plan_service_server_ = this->create_service<lx_msgs::srv::Plan>("task_planner",
                                        std::bind(&TaskPlanner::taskPlannerCallback, this, std::placeholders::_1, std::placeholders::_2));

    
}

void TaskPlanner::taskPlannerCallback(const std::shared_ptr<lx_msgs::srv::Plan::Request> req,
                                          std::shared_ptr<lx_msgs::srv::Plan::Response> res){
    RCLCPP_INFO(this->get_logger(), "Received task planning request");

    // Get points of the berm
    std::vector<geometry_msgs::msg::Point> berm_points = req->berm_input;

    // Loop through the points
    for (int i = 0; i < berm_points.size() - 1; i++){
        // Point 1
        geometry_msgs::msg::Point point_1 = berm_points[i];

        // Point 2
        geometry_msgs::msg::Point point_2 = berm_points[i + 1];

        // Find slope of the line and the center
        double slope = -(point_2.x - point_1.x) / (point_2.y - point_1.y);

        geometry_msgs::msg::Point center;
        center.x = (point_1.x + point_2.x) / 2;
        center.y = (point_1.y + point_2.y) / 2;

        // Add to berm sections
        TaskPlanner::BermSection berm_section(center, slope);
        berm_sections_.push_back(berm_section);
    }

    // For each berm section, there will be an excavation task and a dump task

    /********** EXCAVATION TASK **********/
    // Assuming that the corner near the router is the origin
    // Temporary start and end pose for excavation at (2,2) and (5,2) respectively with orientation of zero degrees throughout
    lx_msgs::msg::PlannedTask excavation_task;
    excavation_task.task_type = lx_msgs::msg::PlannedTask::EXCAVATION;

    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = 2;
    start_pose.position.y = 2;
    excavation_task.pose_array.poses.push_back(start_pose);

    geometry_msgs::msg::Pose end_pose;
    end_pose.position.x = 5;
    end_pose.position.y = 2;
    excavation_task.pose_array.poses.push_back(end_pose);

    // Loop through the berm sections
    for (int i = 0; i < berm_sections_.size(); i++){
        // Get the berm section
        BermSection berm_section = berm_sections_[i];

        // Get the center and slope
        geometry_msgs::msg::Point center = berm_section.center;
        // double slope = berm_section.slope;

        // Assuming a straight berm, the dump pose will be calculated as 50 cm lower than the berm section center in y direction
        geometry_msgs::msg::Pose dump_pose;
        dump_pose.position.x = center.x;
        dump_pose.position.y = center.y - 0.5;
        // Direction should be 90 degrees
        dump_pose.orientation = eulerToQuaternion(0, 0, 1.5708);

        // Add navigation task to the plan
        lx_msgs::msg::PlannedTask navigation_task;
        navigation_task.task_type = lx_msgs::msg::PlannedTask::NAVIGATION;
        navigation_task.pose_array.poses.push_back(start_pose);

        // Add excavation task to the plan
        res->plan.push_back(excavation_task);

        // Add navigation task to the plan
        navigation_task.pose_array.poses.clear();
        navigation_task.pose_array.poses.push_back(dump_pose);

        res->plan.push_back(navigation_task);

        // Add dump task to the plan
        lx_msgs::msg::PlannedTask dump_task;
        dump_task.task_type = lx_msgs::msg::PlannedTask::DUMP;
        dump_task.pose_array.poses.push_back(dump_pose);

        res->plan.push_back(dump_task);

        res->plan.push_back(navigation_task);
    }
}