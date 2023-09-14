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
        RCLCPP_INFO(this->get_logger(), "Parameter set Mobility: %s", (rover_soft_lock_.mobility_lock?"Locked":"Unlocked"));
        rover_soft_lock_.actuation_lock = future.get()->values.at(1).bool_value;
        RCLCPP_INFO(this->get_logger(), "Parameter set Actuation: %s", (rover_soft_lock_.actuation_lock?"Locked":"Unlocked"));

        switch(future.get()->values.at(2).integer_value){
            case 0:
               current_rover_op_mode_ = OpModeEnum::STANDBY;
               RCLCPP_INFO(this->get_logger(), "Parameter set Operation mode: Standby");
               break;
            case 1:
               current_rover_op_mode_ = OpModeEnum::TELEOP;
               RCLCPP_INFO(this->get_logger(), "Parameter set Operation mode: Teleop");
               break;
            case 2:
               current_rover_op_mode_ = OpModeEnum::AUTONOMOUS;
               RCLCPP_INFO(this->get_logger(), "Parameter set Operation mode: Autonomous");
               break;
        }

        switch(future.get()->values.at(3).integer_value){
            case 0:
               current_rover_task_mode_ = TaskModeEnum::IDLE;
               RCLCPP_INFO(this->get_logger(), "Parameter set Task mode: Idle");
               break;
            case 1:
               current_rover_task_mode_ = TaskModeEnum::NAV;
               RCLCPP_INFO(this->get_logger(), "Parameter set Task mode: Navigation");
               break;
            case 2:
               current_rover_task_mode_ = TaskModeEnum::EXC;
               RCLCPP_INFO(this->get_logger(), "Parameter set Task mode: Excavation");
               break;
            case 3:
               current_rover_task_mode_ = TaskModeEnum::DMP;
               RCLCPP_INFO(this->get_logger(), "Parameter set Task mode: Dumping");
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
    // TODO

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
    // TODO

    // Set task mode as NAV
    switchRoverTaskMode(TaskModeEnum::NAV);

    // Call autonav action

    // Block till action complete

    // Set task mode as IDLE
    switchRoverTaskMode(TaskModeEnum::IDLE);

    // Return true if autonav successful
    return true;
}

bool OperationsHandler::callAutoDig(Task current_task){
    // TODO

    // Set task mode as EXC
    switchRoverTaskMode(TaskModeEnum::EXC);

    // Call autodig action

    // Block till action complete

    // Set task mode as IDLE
    switchRoverTaskMode(TaskModeEnum::IDLE);

    // Return true if autodig successful
    return true;
}

bool OperationsHandler::callAutoDump(Task current_task){
    // TODO

    // Set task mode as DMP
    switchRoverTaskMode(TaskModeEnum::DMP);

    // Call autodump action

    // Block till action complete

    // Set task mode as IDLE
    switchRoverTaskMode(TaskModeEnum::IDLE);

    // Return true if autodump successful
    return true;
}

bool OperationsHandler::checkBermBuilt(){
    // TODO

    // Call berm evaluation service

    // Send action feedback on progress

    // Return true if berm is built
    return true;
}