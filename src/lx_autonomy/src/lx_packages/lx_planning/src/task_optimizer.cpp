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
 * */

#include "lx_planning/task_optimizer.hpp"


TaskPlanner::TaskPlanner(): Node("task_planner_node"){
    
    // Set up subscriptions, publishers, servers & clients
    setupCommunications();

    RCLCPP_INFO(this->get_logger(), "Task Planner initialized");
}

void TaskPlanner::setupCommunications(){
    // Subscribers

    // Clients

    // Servers

    // Action servers
    using namespace std::placeholders;
    this->taskplanner_action_server_ = rclcpp_action::create_server<PlanTask>(this, "plan_task",
                                            std::bind(&TaskPlanner::handle_goal, this, _1, _2),
                                            std::bind(&TaskPlanner::handle_cancel, this, _1),
                                            std::bind(&TaskPlanner::handle_accepted, this, _1));

    
}

rclcpp_action::GoalResponse TaskPlanner::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PlanTask::Goal> goal){
    RCLCPP_INFO(this->get_logger(), "Received task planner request");
    (void)uuid;
    (void)goal;
    
    // Accept and execute action
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TaskPlanner::handle_cancel(const std::shared_ptr<GoalHandlePlanTask> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel planner");
    (void)goal_handle;
    
    // Cancel action
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TaskPlanner::handle_accepted(const std::shared_ptr<GoalHandlePlanTask> goal_handle){
    using namespace std::placeholders;
    
    // Start thread to execute action and detach
    std::thread{std::bind(&TaskPlanner::findBermSequence, this, _1), goal_handle}.detach();
}

void TaskPlanner::findBermSequence(const std::shared_ptr<GoalHandlePlanTask> goal_handle) {
    // Clear class variables
    this->berm_inputs_.clear();
    this->berm_section_iterations_.clear();
    this->berm_section_heights_.clear();
    this->excavation_poses_.clear();
    this->map_.data.clear();

    // Get goal, feedback and result
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<PlanTask::Feedback>();
    auto result = std::make_shared<PlanTask::Result>();

    // Update new goal
    this->section_length_ = goal->section_length;
    this->desired_berm_height_ = goal->berm_height;
    this->map_ = goal->map;
    
    // Loop through the berm inputs
    for (int i = 0; i < static_cast<int>(goal->berm_input.size()) - 1; i++){
        // Berm end points
        geometry_msgs::msg::Point point_1 = goal->berm_input[i];
        geometry_msgs::msg::Point point_2 = goal->berm_input[i + 1];

        // Calculate center and angle of berm section
        Pose2D berm_section;
        berm_section.x = (point_1.x + point_2.x) / 2;
        berm_section.y = (point_1.y + point_2.y) / 2;
        berm_section.theta = atan2(point_2.y - point_1.y, point_2.x - point_1.x);

        // Add to berm sequence
        this->berm_inputs_.push_back(berm_section);
    }

    // Loop through the excavation inputs
    for (int i = 0; i < static_cast<int>(goal->excavation_input.size()); i++){
        // Excavation points
        geometry_msgs::msg::Point point = goal->excavation_input[i];

        // Add to excavation sequence
        this->excavation_poses_.push_back(Pose2D(point.x, point.y, 0.0));
    }

    // DON'T EDIT THESE
    // Calculate number of iterations for each berm section
    int num_iterations = static_cast<int>(std::round(std::pow(desired_berm_height_ / INIT_BERM_HEIGHT, 2)));
    berm_section_iterations_.resize(this->berm_inputs_.size(), num_iterations);
    // Set berm heights to zero
    berm_section_heights_.resize(this->berm_inputs_.size(), 0.0);

    // CODE HERE
}

std::vector<Pose2D> TaskPlanner::getDumpPoses(const Pose2D &berm_section) {
    Pose2D dump_pose_1, dump_pose_2;
    std::vector<Pose2D> dump_poses;

    // Calculate dump poses on either side of the berm section with TOOL_DISTANCE_TO_DUMP from the berm section center
    double angle_1 = berm_section.theta + M_PI / 2;
    dump_pose_1.x = berm_section.x + this->TOOL_DISTANCE_TO_DUMP * cos(angle_1);
    dump_pose_1.y = berm_section.y + this->TOOL_DISTANCE_TO_DUMP * sin(angle_1);
    dump_pose_1.theta = angle_1 - M_PI;
    dump_poses.push_back(dump_pose_1);

    double angle_2 = berm_section.theta - M_PI / 2;
    dump_pose_2.x = berm_section.x + this->TOOL_DISTANCE_TO_DUMP * cos(angle_2);
    dump_pose_2.y = berm_section.y + this->TOOL_DISTANCE_TO_DUMP * sin(angle_2);
    dump_pose_2.theta = angle_2 - M_PI;
    dump_poses.push_back(dump_pose_2);

    return dump_poses;
}

/********************************************************************************************************/
// UTILITIES
std::vector<geometry_msgs::msg::Point> TaskPlanner::getRoverFootprint(const Pose2D& pose) {
    // Generate footprint of the rover
    std::vector<geometry_msgs::msg::Point> footprint;
    geometry_msgs::msg::Point point;
    point.x = this->MAX_TOOL_DISTANCE_FROM_BASE;
    point.y = this->ROVER_WIDTH / 2;
    footprint.push_back(point);
    point.x = this->MAX_TOOL_DISTANCE_FROM_BASE;
    point.y = -this->ROVER_WIDTH / 2;
    footprint.push_back(point);
    point.x = -this->ROVER_LENGTH / 2;
    point.y = -this->ROVER_WIDTH / 2;
    footprint.push_back(point);
    point.x = -this->ROVER_LENGTH / 2;
    point.y = this->ROVER_WIDTH / 2;

    // Rotate and translate footprint
    for (int i = 0; i < static_cast<int>(footprint.size()); i++) {
        double x = footprint[i].x;
        double y = footprint[i].y;
        footprint[i].x = x * cos(pose.theta) - y * sin(pose.theta) + pose.x;
        footprint[i].y = x * sin(pose.theta) + y * cos(pose.theta) + pose.y;
    }

    return footprint;
}

Bounds TaskPlanner::getBounds(const std::vector<geometry_msgs::msg::Point>& points) {
    Bounds bounds;

    for (int i = 0; i < static_cast<int>(points.size()); i++) {
        bounds.x_min = std::min(bounds.x_min, points[i].x);
        bounds.x_max = std::max(bounds.x_max, points[i].x);
        bounds.y_min = std::min(bounds.y_min, points[i].y);
        bounds.y_max = std::max(bounds.y_max, points[i].y);
    }

    return bounds;
}