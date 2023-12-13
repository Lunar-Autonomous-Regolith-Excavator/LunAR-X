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


TaskOptimizer::TaskOptimizer(): Node("task_optimizer"){
    
    // Set up subscriptions, publishers, servers & clients
    setupCommunications();

    RCLCPP_INFO(this->get_logger(), "Task Optimizer initialized");
}

void TaskOptimizer::setupCommunications(){
    // Subscribers

    // Clients

    // Servers

    // Action servers
    using namespace std::placeholders;
    this->taskplanner_action_server_ = rclcpp_action::create_server<PlanTask>(this, "plan_task",
                                            std::bind(&TaskOptimizer::handle_goal, this, _1, _2),
                                            std::bind(&TaskOptimizer::handle_cancel, this, _1),
                                            std::bind(&TaskOptimizer::handle_accepted, this, _1));

    
}

rclcpp_action::GoalResponse TaskOptimizer::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PlanTask::Goal> goal){
    RCLCPP_INFO(this->get_logger(), "Received task planner request");
    (void)uuid;
    (void)goal;
    
    // Accept and execute action
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TaskOptimizer::handle_cancel(const std::shared_ptr<GoalHandlePlanTask> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel planner");
    (void)goal_handle;
    
    // Cancel action
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TaskOptimizer::handle_accepted(const std::shared_ptr<GoalHandlePlanTask> goal_handle){
    using namespace std::placeholders;
    
    // Start thread to execute action and detach
    std::thread{std::bind(&TaskOptimizer::findBermSequence, this, _1), goal_handle}.detach();
}

void TaskOptimizer::findBermSequence(const std::shared_ptr<GoalHandlePlanTask> goal_handle) {
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
    
    // Compute berm_section and excavation_poses as vectors of Pose2D
    for (int i = 0; i < static_cast<int>(goal->berm_input.size()) - 1; i++){
        // Berm end points
        geometry_msgs::msg::Point point_1 = goal->berm_input[i];
        geometry_msgs::msg::Point point_2 = goal->berm_input[i + 1];

        // Check the distance between the two points
        double distance = std::sqrt(std::pow(point_2.x - point_1.x, 2) + std::pow(point_2.y - point_1.y, 2));
        if (abs(distance - this->section_length_) > 0.1)
        {
            cout << "Ignoring berm section " << i << endl;
            continue;
        }

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
        this->excavation_poses_.push_back(Pose2D(point.x, point.y, point.z));
    }

    // Calculate number of iterations for each berm section
    int num_dumps_per_segment = static_cast<int>(std::round(std::pow(desired_berm_height_ / INIT_BERM_HEIGHT, 2)));
    berm_section_iterations_.resize(this->berm_inputs_.size(), num_dumps_per_segment);

    // Set berm heights to zero
    berm_section_heights_.resize(this->berm_inputs_.size(), 0.0);

    // Call optimization function
    OptimalSequencePlanner optimal_sequence_planner = OptimalSequencePlanner(this->map_, this->berm_inputs_, this->excavation_poses_, num_dumps_per_segment, this->section_length_, this->desired_berm_height_);
    vector<lx_msgs::msg::PlannedTask> plan = optimal_sequence_planner.get_plan();

    // Return result
    result->plan = plan;
    goal_handle->succeed(result);
}

std::vector<Pose2D> TaskOptimizer::getDumpPoses(const Pose2D &berm_section) {
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
