/* Author: Dhruv Tyagi
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

    // Get parameters from the global parameter server
    getParams();

    // Set up parameters from the global parameter server
    setupParams();


    RCLCPP_INFO(this->get_logger(), "Operations handler initialized");
}

void OperationsHandler::getParams(){
    while(!get_params_client_->wait_for_service(std::chrono::seconds(2))){
      RCLCPP_INFO(this->get_logger(), "Could not contact param server");
      return;
    }
    // Get important parameters
    auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    get_request->names = {"rover.mobility_lock", "rover.actuation_lock", 
                          "rover.op_mode", "rover.task_mode"};
    // Send request
    auto param_result_ = get_params_client_->async_send_request(get_request,std::bind(&OperationsHandler::paramCB, this, std::placeholders::_1));
}

void OperationsHandler::paramCB(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future){
    auto status = future.wait_for(std::chrono::milliseconds(100));
    // If request successful, save all params in global variables
    if (status == std::future_status::ready) {
        // params_timer_ = this->create_wall_timer(std::chrono::seconds(10), 
        //                     std::bind(&LXGUIBackend::getParameters, this));
        
        rover_soft_lock_.mobility_lock = future.get()->values.at(0).bool_value;
        RCLCPP_DEBUG(this->get_logger(), "Parameter set Mobility: %s", (rover_soft_lock_.mobility_lock?"Locked":"Unlocked"));
        rover_soft_lock_.actuation_lock = future.get()->values.at(1).bool_value;
        RCLCPP_DEBUG(this->get_logger(), "Parameter set Actuation: %s", (rover_soft_lock_.actuation_lock?"Locked":"Unlocked"));

        switch(future.get()->values.at(2).integer_value){
            case 0:
               current_rover_op_mode_ = OpModeEnum::STANDBY;
               RCLCPP_DEBUG(this->get_logger(), "Parameter set Operation mode: Standby");
               break;
            case 1:
               current_rover_op_mode_ = OpModeEnum::TELEOP;
               RCLCPP_DEBUG(this->get_logger(), "Parameter set Operation mode: Teleop");
               break;
            case 2:
               current_rover_op_mode_ = OpModeEnum::AUTONOMOUS;
               RCLCPP_DEBUG(this->get_logger(), "Parameter set Operation mode: Autonomous");
               break;
        }

        switch(future.get()->values.at(3).integer_value){
            case 0:
               current_rover_task_mode_ = TaskModeEnum::IDLE;
               RCLCPP_DEBUG(this->get_logger(), "Parameter set Task mode: Idle");
               break;
            case 1:
               current_rover_task_mode_ = TaskModeEnum::NAV;
               RCLCPP_DEBUG(this->get_logger(), "Parameter set Task mode: Navigation");
               break;
            case 2:
               current_rover_task_mode_ = TaskModeEnum::EXC;
               RCLCPP_DEBUG(this->get_logger(), "Parameter set Task mode: Excavation");
               break;
            case 3:
               current_rover_task_mode_ = TaskModeEnum::DMP;
               RCLCPP_DEBUG(this->get_logger(), "Parameter set Task mode: Dumping");
               break;
        }
    } 
    else {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}

void OperationsHandler::setupCommunications(){
    // Add all subscriptions, publishers and services here

    // Clients
    set_params_client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/param_server_node/set_parameters");
    get_params_client_ = this->create_client<rcl_interfaces::srv::GetParameters>("/param_server_node/get_parameters");
    // Action server
    using namespace std::placeholders;
    this->operation_action_server_ = rclcpp_action::create_server<Operation>(this, "operations/berm_build_action",
                                            std::bind(&OperationsHandler::handle_goal, this, _1, _2),
                                            std::bind(&OperationsHandler::handle_cancel, this, _1),
                                            std::bind(&OperationsHandler::handle_accepted, this, _1));
    // Action clients
    this->auto_nav_action_client_ = rclcpp_action::create_client<AutoNav>(this, "operations/autonav_action");
    this->auto_dig_action_client_ = rclcpp_action::create_client<AutoDig>(this, "operations/autodig_action");
    this->auto_dump_action_client_ = rclcpp_action::create_client<AutoDump>(this, "operations/autodump_action");
}

void OperationsHandler::setupParams(){
    // Subscriber for global parameter events
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // Callback Lambdas
    auto mob_params_callback = [this](const rclcpp::Parameter & p) {
        RCLCPP_DEBUG(this->get_logger(), "Parameter updated \"%s\": %s", p.get_name().c_str(), (p.as_bool()?"Locked":"Unlocked"));
        rover_soft_lock_.mobility_lock = p.as_bool();
    };
    auto act_params_callback = [this](const rclcpp::Parameter & p) {
        RCLCPP_DEBUG(this->get_logger(), "Parameter updated \"%s\": %s", p.get_name().c_str(), (p.as_bool()?"Locked":"Unlocked"));
        rover_soft_lock_.actuation_lock = p.as_bool();
    };
    auto op_mode_params_callback = [this](const rclcpp::Parameter & p) {
        switch(p.as_int()){
            case 0:
               current_rover_op_mode_ = OpModeEnum::STANDBY;
               RCLCPP_DEBUG(this->get_logger(), "Parameter updated \"%s\": Standby", p.get_name().c_str());
               break;
            case 1:
               current_rover_op_mode_ = OpModeEnum::TELEOP;
               RCLCPP_DEBUG(this->get_logger(), "Parameter updated \"%s\": Teleop", p.get_name().c_str());
               break;
            case 2:
               current_rover_op_mode_ = OpModeEnum::AUTONOMOUS;
               RCLCPP_DEBUG(this->get_logger(), "Parameter updated \"%s\": Autonomous", p.get_name().c_str());
               break;
        }
    };
    auto task_mode_params_callback = [this](const rclcpp::Parameter & p){
        switch(p.as_int()){
            case 0:
               current_rover_task_mode_ = TaskModeEnum::IDLE;
               RCLCPP_DEBUG(this->get_logger(), "Parameter updated \"%s\": Idle", p.get_name().c_str());
               break;
            case 1:
               current_rover_task_mode_ = TaskModeEnum::NAV;
               RCLCPP_DEBUG(this->get_logger(), "Parameter updated \"%s\": Navigation", p.get_name().c_str());
               break;
            case 2:
               current_rover_task_mode_ = TaskModeEnum::EXC;
               RCLCPP_DEBUG(this->get_logger(), "Parameter updated \"%s\": Excavation", p.get_name().c_str());
               break;
            case 3:
               current_rover_task_mode_ = TaskModeEnum::DMP;
               RCLCPP_DEBUG(this->get_logger(), "Parameter updated \"%s\": Dumping", p.get_name().c_str());
               break;
        }
    };

    // Names of node & params for adding callback
    auto param_server_name = std::string("param_server_node");
    auto mob_lock_param_name = std::string("rover.mobility_lock");
    auto act_lock_param_name = std::string("rover.actuation_lock");
    auto op_mode_param_name = std::string("rover.op_mode");
    auto task_mode_param_name = std::string("rover.task_mode");

    // Store callback handles for each parameter
    mob_param_cb_handle_ = param_subscriber_->add_parameter_callback(mob_lock_param_name, mob_params_callback, param_server_name);
    act_param_cb_handle_ = param_subscriber_->add_parameter_callback(act_lock_param_name, act_params_callback, param_server_name);
    op_mode_param_cb_handle_ = param_subscriber_->add_parameter_callback(op_mode_param_name, op_mode_params_callback, param_server_name);
    task_mode_param_cb_handle_ = param_subscriber_->add_parameter_callback(task_mode_param_name, task_mode_params_callback, param_server_name);
}

void OperationsHandler::switchRoverTaskMode(TaskModeEnum mode_to_set){
    while(!set_params_client_->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_WARN(this->get_logger(), "Waiting for params server to be up...");
    }

    auto set_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    auto task_mode_param_req = rcl_interfaces::msg::Parameter();
    task_mode_param_req.name = "rover.task_mode";
    task_mode_param_req.value.type = 2;
    task_mode_param_req.value.integer_value = uint16_t(mode_to_set);

    set_request->parameters = {task_mode_param_req};

    auto future_result = set_params_client_->async_send_request(set_request);
}

rclcpp_action::GoalResponse OperationsHandler::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Operation::Goal> goal){
    // Get requested berm configuration
    berm_config_.berm_configuration = goal->requested_berm_config.berm_configuration;
    
    RCLCPP_INFO(this->get_logger(), "Received berm goal request with configuration:");
    for(auto &berm_node: berm_config_.berm_configuration){
        RCLCPP_INFO(this->get_logger(), "%.2f, %.2f", berm_node.x, berm_node.y);
    }
    (void)uuid;
    
    // Accept and execute action
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OperationsHandler::handle_cancel(const std::shared_ptr<GoalHandleOperation> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    
    // Cancel action
    return rclcpp_action::CancelResponse::ACCEPT;
}

void OperationsHandler::handle_accepted(const std::shared_ptr<GoalHandleOperation> goal_handle){
    using namespace std::placeholders;
    
    // Start thread to execute action and detach
    std::thread{std::bind(&OperationsHandler::executeOperation, this, _1), goal_handle}.detach();
}

void OperationsHandler::executeOperation(const std::shared_ptr<GoalHandleOperation> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Executing operation goal");
    
    // Get goal, feedback and result
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Operation::Feedback>();
    auto result = std::make_shared<Operation::Result>();

    do{
        // Set task mode as IDLE
        switchRoverTaskMode(TaskModeEnum::IDLE);

        // Call planner to plan full path
        // Get task queue from planner
        task_queue_ = planner();

        // TODO Only keep first N tasks in queue

        // Execute task queue
        if(!executeTaskQueue()){
            // If any task fails, return goal failure
            result->success = false;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Operations goal failed");
            return;
        }
        // If all tasks successful, continue next loop iteration
    } 
    while(!checkBermBuilt());

    switchRoverTaskMode(TaskModeEnum::IDLE);
    
    // If berm is built, return goal success
    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Operations goal succeeded, berm built");
    }
}

std::queue<Task, std::list<Task>> OperationsHandler::planner(){
    // TODO

    // Decide planner inputs

    // Planner should give task ids. Check already executed tasks by accessing executed_task_ids_

    std::queue<Task, std::list<Task>> build_task_queue {};

    // Add tasks to task queue

    // TEST TASKS - TO BE REMOVED
    auto test_pose = geometry_msgs::msg::PoseArray();
    build_task_queue.push(Task(0, TaskTypeEnum::AUTONAV, test_pose));
    build_task_queue.push(Task(1, TaskTypeEnum::AUTODIG, test_pose));
    build_task_queue.push(Task(2, TaskTypeEnum::AUTODUMP, test_pose));

    return build_task_queue;
}

bool OperationsHandler::executeTaskQueue(){
    // Execute task queue
    while(!task_queue_.empty()){
        // Check if rover locked
        if(rover_soft_lock_.mobility_lock || rover_soft_lock_.actuation_lock){
            RCLCPP_ERROR(this->get_logger(), "Rover locked - Task queue can not be executed");
            return false;
        }
        else{
            // Execute task
            Task current_task = task_queue_.front();
            task_queue_.pop();
            switch(current_task.getType()){
                case TaskTypeEnum::AUTONAV:
                    if(!callAutoNav(current_task)){
                        return false;
                    }
                    break;
                case TaskTypeEnum::AUTODIG:
                    if(!callAutoDig(current_task)){
                        return false;
                    }
                    break;
                case TaskTypeEnum::AUTODUMP:
                    if(!callAutoDump(current_task)){
                        return false;
                    }
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Invalid task type");
                    return false;
            }
            // Append executed task id to executed_task_ids_
            executed_task_ids_.push_back(current_task.getID());
        }
    }

    return true;
}

bool OperationsHandler::callAutoNav(Task current_task){
    // Call autonav action
    using namespace std::placeholders;
    if (!auto_nav_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Auto Nav action server not available after waiting");
        // Set task mode as IDLE
        switchRoverTaskMode(TaskModeEnum::IDLE);
        return false;
    }
    // Block till action returns result or rejection
    auto_action_blocking_ = true;
    auto_action_server_responded_ = false;
    auto_action_accepted_ = false;
    auto_action_success_ = false;

    auto autonav_request_msg = AutoNav::Goal();
    // TODO Create Autonav Request from the Task
    (void)current_task;
    
    RCLCPP_INFO(this->get_logger(), "Sending Auto Nav goal");
    
    auto send_goal_options = rclcpp_action::Client<AutoNav>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&OperationsHandler::autoNavResponseCB, this, _1);
    send_goal_options.feedback_callback = std::bind(&OperationsHandler::autoNavFeedbackCB, this, _1, _2);
    send_goal_options.result_callback = std::bind(&OperationsHandler::autoNavResultCB, this, _1);

    auto goal_handle_future = auto_nav_action_client_->async_send_goal(autonav_request_msg, send_goal_options);

    // Block till action complete
    rclcpp::Rate loop_rate(1);
    rclcpp::Time action_start_time = this->get_clock()->now();
    while(auto_action_blocking_ && rclcpp::ok() && (this->get_clock()->now() - action_start_time).seconds() < blocking_time_limit_){
        loop_rate.sleep();
    }

    // Set task mode as IDLE
    switchRoverTaskMode(TaskModeEnum::IDLE);

    // Return true if autonav successful
    if(!auto_action_accepted_){
        return false;
    }
    else if(auto_action_success_){
        return true;
    }

    return false;
}

bool OperationsHandler::callAutoDig(Task current_task){
    // Set task mode as EXC
    switchRoverTaskMode(TaskModeEnum::EXC);

    // Call autodig action
    using namespace std::placeholders;
    if (!auto_dig_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Auto Dig action server not available after waiting");
        // Set task mode as IDLE
        switchRoverTaskMode(TaskModeEnum::IDLE);
        return false;
    }
    // Block till action returns result or rejection
    auto_action_blocking_ = true;
    auto_action_server_responded_ = false;
    auto_action_accepted_ = false;
    auto_action_success_ = false;

    auto autodig_request_msg = AutoDig::Goal();
    // TODO Create Autodig Request from the Task
    (void)current_task;

    RCLCPP_INFO(this->get_logger(), "Sending Auto Dig goal");
    
    auto send_goal_options = rclcpp_action::Client<AutoDig>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&OperationsHandler::autoDigResponseCB, this, _1);
    send_goal_options.feedback_callback = std::bind(&OperationsHandler::autoDigFeedbackCB, this, _1, _2);
    send_goal_options.result_callback = std::bind(&OperationsHandler::autoDigResultCB, this, _1);

    auto goal_handle_future = auto_dig_action_client_->async_send_goal(autodig_request_msg, send_goal_options);

    // Block till action complete
    rclcpp::Rate loop_rate(1);
    rclcpp::Time action_start_time = this->get_clock()->now();
    while(auto_action_blocking_ && rclcpp::ok() && (this->get_clock()->now() - action_start_time).seconds() < blocking_time_limit_){
        loop_rate.sleep();
    }

    // Set task mode as IDLE
    switchRoverTaskMode(TaskModeEnum::IDLE);

    // Return true if autodig successful
    if(!auto_action_accepted_){
        return false;
    }
    else if(auto_action_success_){
        return true;
    }

    return false;
}

bool OperationsHandler::callAutoDump(Task current_task){
    // Set task mode as DMP
    switchRoverTaskMode(TaskModeEnum::DMP);

    // Call autodump action
    using namespace std::placeholders;
    if (!auto_dump_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Auto Dump action server not available after waiting");
        // Set task mode as IDLE
        switchRoverTaskMode(TaskModeEnum::IDLE);
        return false;
    }
    // Block till action returns result or rejection
    auto_action_blocking_ = true;
    auto_action_server_responded_ = false;
    auto_action_accepted_ = false;
    auto_action_success_ = false;

    auto autodump_request_msg = AutoDump::Goal();
    // TODO Create Autodump Request from the Task
    (void)current_task;

    RCLCPP_INFO(this->get_logger(), "Sending Auto Dump goal");
    
    auto send_goal_options = rclcpp_action::Client<AutoDump>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&OperationsHandler::autoDumpResponseCB, this, _1);
    send_goal_options.feedback_callback = std::bind(&OperationsHandler::autoDumpFeedbackCB, this, _1, _2);
    send_goal_options.result_callback = std::bind(&OperationsHandler::autoDumpResultCB, this, _1);

    auto goal_handle_future = auto_dump_action_client_->async_send_goal(autodump_request_msg, send_goal_options);

    // Block till action complete
    rclcpp::Rate loop_rate(1);
    rclcpp::Time action_start_time = this->get_clock()->now();
    while(auto_action_blocking_ && rclcpp::ok() && (this->get_clock()->now() - action_start_time).seconds() < blocking_time_limit_){
        loop_rate.sleep();
    }

    // Set task mode as IDLE
    switchRoverTaskMode(TaskModeEnum::IDLE);

    // Return true if autodig successful
    if(!auto_action_accepted_){
        return false;
    }
    else if(auto_action_success_){
        return true;
    }

    return false;
}

void OperationsHandler::autoNavResponseCB(GoalHandleAutoNav::SharedPtr future){
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Auto nav goal was rejected by server");
        auto_action_blocking_ = false;
        auto_action_server_responded_ = true;
        auto_action_accepted_ = false;
        auto_action_success_ = false;
    } else {
        RCLCPP_INFO(this->get_logger(), "Auto nav goal accepted by server, waiting for result");
        auto_action_server_responded_ = true;
        auto_action_accepted_ = true;
    }
}

void OperationsHandler::autoNavFeedbackCB(GoalHandleAutoNav::SharedPtr, const std::shared_ptr<const AutoNav::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Received feedback: %s", feedback->status.c_str());
}

void OperationsHandler::autoNavResultCB(const GoalHandleAutoNav::WrappedResult & result){
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Auto nav goal succeeded");
            auto_action_success_ = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Auto nav goal was aborted");
            auto_action_success_ = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Auto nav goal was canceled");
            auto_action_success_ = false;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            auto_action_success_ = false;
            break;
    }
    auto_action_blocking_ = false;
}

void OperationsHandler::autoDigResponseCB(GoalHandleAutoDig::SharedPtr future){
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Auto dig goal was rejected by server");
        auto_action_blocking_ = false;
        auto_action_server_responded_ = true;
        auto_action_accepted_ = false;
        auto_action_success_ = false;
    } else {
        RCLCPP_INFO(this->get_logger(), "Auto dig goal accepted by server, waiting for result");
        auto_action_server_responded_ = true;
        auto_action_accepted_ = true;
    }
}

void OperationsHandler::autoDigFeedbackCB(GoalHandleAutoDig::SharedPtr, const std::shared_ptr<const AutoDig::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Received feedback: %s", feedback->status.c_str());
}

void OperationsHandler::autoDigResultCB(const GoalHandleAutoDig::WrappedResult & result){
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Auto dig goal succeeded");
            auto_action_success_ = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Auto dig goal was aborted");
            auto_action_success_ = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Auto dig goal was canceled");
            auto_action_success_ = false;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            auto_action_success_ = false;
            break;
    }
    auto_action_blocking_ = false;
}

void OperationsHandler::autoDumpResponseCB(GoalHandleAutoDump::SharedPtr future){
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Auto dump goal was rejected by server");
        auto_action_blocking_ = false;
        auto_action_server_responded_ = true;
        auto_action_accepted_ = false;
        auto_action_success_ = false;
    } else {
        RCLCPP_INFO(this->get_logger(), "Auto dump goal accepted by server, waiting for result");
        auto_action_server_responded_ = true;
        auto_action_accepted_ = true;
    }
}

void OperationsHandler::autoDumpFeedbackCB(GoalHandleAutoDump::SharedPtr, const std::shared_ptr<const AutoDump::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Received feedback: %s", feedback->status.c_str());
}

void OperationsHandler::autoDumpResultCB(const GoalHandleAutoDump::WrappedResult & result){
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Auto dump goal succeeded");
            auto_action_success_ = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Auto dump goal was aborted");
            auto_action_success_ = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Auto dump goal was canceled");
            auto_action_success_ = false;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            auto_action_success_ = false;
            break;
    }
    auto_action_blocking_ = false;
}

bool OperationsHandler::checkBermBuilt(){
    // TODO

    // Call berm evaluation service

    // Send action feedback on progress

    // Return true if berm is built
    return true;
}