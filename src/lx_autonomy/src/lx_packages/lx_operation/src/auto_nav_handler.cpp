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
 * - Add todos
 * */

#include "lx_operation/auto_nav_handler.hpp"

AutoNavHandler::AutoNavHandler(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()): Node("auto_nav_handler_node"){
    (void)options;

    // Set up subscriptions, publishers, services, action servers and clients
    setupCommunications();

    // Get parameters from the global parameter server
    getParams();

    // Set up parameters from the global parameter server
    setupParams();


    RCLCPP_INFO(this->get_logger(), "AutoNav handler initialized");
}

void AutoNavHandler::getParams(){
    while(!get_params_client_->wait_for_service(std::chrono::seconds(2))){
      RCLCPP_INFO(this->get_logger(), "Could not contact param server");
      return;
    }
    // Get important parameters
    auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    get_request->names = {"rover.mobility_lock", "rover.actuation_lock", 
                          "rover.op_mode", "rover.task_mode"};
    // Send request
    auto param_result_ = get_params_client_->async_send_request(get_request,std::bind(&AutoNavHandler::paramCB, this, std::placeholders::_1));
}

void AutoNavHandler::paramCB(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future){
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

void AutoNavHandler::setupCommunications(){
    // Service clients
    get_params_client_ = this->create_client<rcl_interfaces::srv::GetParameters>("/lx_param_server_node/get_parameters");
    
    // Action server
    using namespace std::placeholders;
    this->autonav_action_server_ = rclcpp_action::create_server<AutoNav>(this, "operations/autonav_action",
                                            std::bind(&AutoNavHandler::handle_goal, this, _1, _2),
                                            std::bind(&AutoNavHandler::handle_cancel, this, _1),
                                            std::bind(&AutoNavHandler::handle_accepted, this, _1));
    
    // Action client
    this->compute_path_client_ = rclcpp_action::create_client<ComputePathToPose>(this, "compute_path_to_pose");
    this->follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");

    // Subscribers
    this->cmd_vel_nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_nav", 10, std::bind(&AutoNavHandler::cmdVelNavCallback, this, _1));
    
    // Publishers
    this->rover_cmd_pub_ = this->create_publisher<lx_msgs::msg::RoverCommand>("rover_auto_cmd", 10);
}

void AutoNavHandler::setupParams(){
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
    auto param_server_name = std::string("lx_param_server_node");
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

rclcpp_action::GoalResponse AutoNavHandler::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const AutoNav::Goal> goal){
    RCLCPP_INFO(this->get_logger(), "Received autonav request");
    (void)uuid;
    (void)goal;
    
    // Accept and execute action
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AutoNavHandler::handle_cancel(const std::shared_ptr<GoalHandleAutoNav> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    
    // Cancel action
    return rclcpp_action::CancelResponse::ACCEPT;
}

void AutoNavHandler::handle_accepted(const std::shared_ptr<GoalHandleAutoNav> goal_handle){
    using namespace std::placeholders;
    
    // Start thread to execute action and detach
    std::thread{std::bind(&AutoNavHandler::executeAutoNav, this, _1), goal_handle}.detach();
}

void AutoNavHandler::executeAutoNav(const std::shared_ptr<GoalHandleAutoNav> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Executing autonav request");
    
    // Get goal, feedback and result
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<AutoNav::Feedback>();
    auto result = std::make_shared<AutoNav::Result>();

    // Clear nav2 variables
    this->path_ = nav_msgs::msg::Path();
    this->planning_time_ = builtin_interfaces::msg::Duration();
    
    if (!this->computePath(goal->goal)) {
        result->success = false;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Autonav failed");
        return;
    }

    // Print planning time in ms
    RCLCPP_INFO(this->get_logger(), "Planning time: %f ms", this->planning_time_.sec * 1000 + this->planning_time_.nanosec / 1000000.0);

    // Follow path
    if (!this->followPath()) {
        result->success = false;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Autonav failed");
        return;
    }
    
    // If autonav executed successfully, return goal success
    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Autonav succeeded");
    }
}

bool AutoNavHandler::computePath(const geometry_msgs::msg::PoseStamped& goal_pose){
    using namespace std::placeholders;
    if (!this->compute_path_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Compute path action server not available after waiting");
        return false;
    }
    
    // Block action until path is computed
    this->planner_action_blocking_ = true;
    this->planner_action_server_responded_ = false;
    this->planner_action_accepted_ = false;
    this->planner_action_success_ = false;
    
    // Create goal message
    auto goal_msg = ComputePathToPose::Goal();
    goal_msg.goal = goal_pose;
    goal_msg.planner_id = "GridBased";
    goal_msg.use_start = false; // No need to provide start pose -- it will use the current robot pose

    RCLCPP_INFO(this->get_logger(), "Sending goal to compute path action server");
    
    auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&AutoNavHandler::computePathResponseCallback, this, _1);
    send_goal_options.result_callback = std::bind(&AutoNavHandler::computePathResultCallback, this, _1);

    auto goal_handle_future = this->compute_path_client_->async_send_goal(goal_msg, send_goal_options);

    // Block till action complete
    rclcpp::Rate loop_rate(20);
    while(this->planner_action_blocking_ && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Computing path...");
        loop_rate.sleep();
    }

    if (!this->planner_action_accepted_) {
        RCLCPP_ERROR(this->get_logger(), "Compute path action was rejected");
        return false;
    }
    else if (!this->planner_action_success_) {
        RCLCPP_ERROR(this->get_logger(), "Compute path action failed");
        return false;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Compute path action succeeded");
        return true;
    }

    return false;
}

bool AutoNavHandler::followPath(){
    using namespace std::placeholders;
    if (!this->follow_path_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Follow path action server not available after waiting");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Inside follow path");
    
    // Block action until path is followed
    this->controller_action_blocking_ = true;
    this->controller_action_server_responded_ = false;
    this->controller_action_accepted_ = false;
    this->controller_action_success_ = false;
    
    // Create goal message
    auto goal_msg = FollowPath::Goal();
    goal_msg.path = path_;
    goal_msg.controller_id = "FollowPath";
    goal_msg.goal_checker_id = "goal_checker";

    RCLCPP_INFO(this->get_logger(), "Sending goal to follow path action server");
    
    auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&AutoNavHandler::followPathResponseCallback, this, _1);
    send_goal_options.result_callback = std::bind(&AutoNavHandler::followPathResultCallback, this, _1);

    auto goal_handle_future = this->follow_path_client_->async_send_goal(goal_msg, send_goal_options);

    // Block till action complete
    rclcpp::Rate loop_rate(10);
    while(this->controller_action_blocking_ && rclcpp::ok()) {
        // Print feedback
        RCLCPP_INFO(this->get_logger(), "In progress... Distance to goal: %f, Speed: %f", this->distance_to_goal_, this->rov_speed_);
        loop_rate.sleep();
    }

    if (!this->controller_action_accepted_) {
        RCLCPP_ERROR(this->get_logger(), "Follow path action was rejected");
        return false;
    }
    else if (!this->controller_action_success_) {
        RCLCPP_ERROR(this->get_logger(), "Follow path action failed");
        return false;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Follow path action succeeded");
        return true;
    }

    return false;
}

void AutoNavHandler::computePathResponseCallback(GoalHandleComputePathToPose::SharedPtr future){
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        this->planner_action_blocking_ = false;
        this->planner_action_server_responded_ = true;
        this->planner_action_accepted_ = false;
        this->planner_action_success_ = false;
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        this->planner_action_server_responded_ = true;
        this->planner_action_accepted_ = true;
    }
}

void AutoNavHandler::computePathResultCallback(const GoalHandleComputePathToPose::WrappedResult & result){
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Path computed successfully");
            this->planner_action_success_ = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            this->planner_action_success_ = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            this->planner_action_success_ = false;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            this->planner_action_success_ = false;
            break;
    }
    action_blocking_ = false;
    if (action_success_) {
        this->path_ = result.result->path;
        this->planning_time_ = result.result->planning_time;
    }
}

void AutoNavHandler::followPathResponseCallback(GoalHandleFollowPath::SharedPtr future){
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        this->controller_action_blocking_ = false;
        this->controller_action_server_responded_ = true;
        this->controller_action_accepted_ = false;
        this->controller_action_success_ = false;
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        this->controller_action_server_responded_ = true;
        this->controller_action_accepted_ = true;
    }
}

void AutoNavHandler::followPathFeedbackCallback(GoalHandleFollowPath::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback){
    this->distance_to_goal_ = feedback->distance_to_goal;
    this->rov_speed_ = feedback->speed;
}

void AutoNavHandler::followPathResultCallback(const GoalHandleFollowPath::WrappedResult & result){
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Path followed successfully");
            this->controller_action_success_ = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            this->controller_action_success_ = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            this->controller_action_success_ = false;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            this->controller_action_success_ = false;
            break;
    }
    action_blocking_ = false;
}

void AutoNavHandler::cmdVelNavCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    lx_msgs::msg::RoverCommand rov_cmd;
    rov_cmd.mobility_twist = *msg;
    
    // Publish rover command
    rover_cmd_pub_->publish(rov_cmd);
}