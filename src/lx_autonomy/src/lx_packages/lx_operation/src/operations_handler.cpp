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
 * - Check operation action
 * - Write planner
 * - Write executeTaskQueue
 * - Write checkBermBuilt
 * - Finish executeOperation
 * - Add documentation
 * - Add parameters
 * */

#include "lx_operation/operations_handler.hpp"

OperationsHandler::OperationsHandler(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()): Node("operations_handler_node"){
    (void)options;
    // Clear executed task queue
    executed_task_ids_.clear();
    
    // Set up subscriptions, publishers, services, action servers and clients
    setupCommunications();


    RCLCPP_INFO(this->get_logger(), "Operations handler initialized");
}

void OperationsHandler::setupCommunications(){
    // Add all subscriptions, publishers and services here

    // Action server
    using namespace std::placeholders;
    this->operation_action_server_ = rclcpp_action::create_server<Operation>(this, "operations/berm_build_action",
                                            std::bind(&OperationsHandler::handle_goal, this, _1, _2),
                                            std::bind(&OperationsHandler::handle_cancel, this, _1),
                                            std::bind(&OperationsHandler::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse OperationsHandler::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Operation::Goal> goal){
    berm_config_.berm_configuration = goal->requested_berm_config.berm_configuration;
    RCLCPP_INFO(this->get_logger(), "Received berm goal request with configuration:");
    for(auto &berm_node: berm_config_.berm_configuration){
        RCLCPP_INFO(this->get_logger(), "%f, %f", berm_node.x, berm_node.y);
    }
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OperationsHandler::handle_cancel(const std::shared_ptr<GoalHandleOperation> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void OperationsHandler::handle_accepted(const std::shared_ptr<GoalHandleOperation> goal_handle){
    using namespace std::placeholders;
    std::thread{std::bind(&OperationsHandler::executeOperation, this, _1), goal_handle}.detach();
}

void OperationsHandler::executeOperation(const std::shared_ptr<GoalHandleOperation> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Executing operation goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Operation::Feedback>();
    auto result = std::make_shared<Operation::Result>();

    do{
        // Call planner to plan full path
        // Get task queue from planner
        task_queue_ = planner();

        // Only keep first N tasks in queue

        // Execute task queue
        if(!executeTaskQueue()){
            // If any task fails, return goal failure
        }
        
        // If all tasks successful, continue next loop iteration
    } 
    while(!checkBermBuilt());
    
    // If berm is built, return goal success

    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Operations goal succeeded, berm built");
    }
}

std::queue<std::shared_ptr<Task>, std::list<std::shared_ptr<Task>>> OperationsHandler::planner(){
    // TODO

    // Decide planner inputs

    // Planner should give task ids. Check already executed tasks by accessing executed_task_ids_

    std::queue<std::shared_ptr<Task>, std::list<std::shared_ptr<Task>>> build_task_queue {};

    // Add tasks to task queue

    return build_task_queue;
}

bool OperationsHandler::executeTaskQueue(){
    // TODO

    // Execute task queue

    // Append successful tasks to executed_task_ids_
    return true;
}

bool checkBermBuilt(){
    // Call berm evaluation service

    // Send action feedback on progress

    // Return true if berm is built
    return true;
}