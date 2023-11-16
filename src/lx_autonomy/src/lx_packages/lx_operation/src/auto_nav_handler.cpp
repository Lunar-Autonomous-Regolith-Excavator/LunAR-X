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
    this->navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

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

    // Update new goal
    this->goal_pose_ = goal->goal;
    
    // Navigate to goal pose
    if (!this->navigateToPose()) {
        result->success = false;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Autonav failed");
        // Publish 0 rover command to stop rover
        lx_msgs::msg::RoverCommand rov_cmd;
        rover_cmd_pub_->publish(rov_cmd);
        return;
    }

    // Publish 0 rover command to stop rover
    lx_msgs::msg::RoverCommand rov_cmd;
    rover_cmd_pub_->publish(rov_cmd);

    // If autonav executed successfully, return goal success
    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Autonav succeeded");
    }
}

bool AutoNavHandler::navigateToPose(){
    using namespace std::placeholders;
    if (!this->navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Navigate to pose action server not available after waiting");
        return false;
    }
    
    // Block action
    this->action_blocking_ = true;
    this->action_server_responded_ = false;
    this->action_accepted_ = false;
    this->action_success_ = false;
    
    // Create goal message
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = this->goal_pose_;
    goal_msg.behavior_tree = "/home/ubuntu/lx_station_ws/src/lx_nav2/config/replanning.xml";
    
    RCLCPP_INFO(this->get_logger(), "Sending goal to navigate to pose action server");
    
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&AutoNavHandler::navigateToPoseResponseCallback, this, _1);
    send_goal_options.feedback_callback = std::bind(&AutoNavHandler::navigateToPoseFeedbackCallback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&AutoNavHandler::navigateToPoseResultCallback, this, _1);

    auto goal_handle_future = this->navigate_to_pose_client_->async_send_goal(goal_msg, send_goal_options);

    auto start_time = this->now();

    // Block till action complete
    rclcpp::Rate loop_rate(10);
    while(this->action_blocking_ && rclcpp::ok()) {
        // Print feedback
        // RCLCPP_INFO(this->get_logger(), "In progress... Distance to goal: %f m | Time remaining: %f secs", this->distance_remaining_, this->estimated_time_remaining_.sec);

        // Check timeout
        // if (this->now() - start_time > rclcpp::Duration(this->MAX_DURATION, 0)) {
        //     RCLCPP_ERROR(this->get_logger(), "AutoNav timed out");
        //     // Cancel goal
        //     this->navigate_to_pose_client_->async_cancel_all_goals();
        // }

        loop_rate.sleep();
    }

    if (!this->action_accepted_) {
        RCLCPP_ERROR(this->get_logger(), "Action was rejected");
        return false;
    }
    else if (!this->action_success_) {
        RCLCPP_ERROR(this->get_logger(), "Action failed");
        return false;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Action succeeded");
        return true;
    }

    return false;
}

void AutoNavHandler::navigateToPoseResponseCallback(GoalHandleNavigateToPose::SharedPtr future){
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        this->action_blocking_ = false;
        this->action_server_responded_ = true;
        this->action_accepted_ = false;
        this->action_success_ = false;
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        this->action_server_responded_ = true;
        this->action_accepted_ = true;
    }
}

void AutoNavHandler::navigateToPoseFeedbackCallback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback){
    this->current_pose_ = feedback->current_pose;
    this->navigation_time_ = feedback->navigation_time;
    this->estimated_time_remaining_ = feedback->estimated_time_remaining;
    this->number_of_recoveries_ = feedback->number_of_recoveries;
    this->distance_remaining_ = feedback->distance_remaining;
}

void AutoNavHandler::navigateToPoseResultCallback(const GoalHandleNavigateToPose::WrappedResult & result){
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Path followed successfully");
            this->action_success_ = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            this->action_success_ = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            this->action_success_ = false;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            this->action_success_ = false;
            break;
    }
    this->action_blocking_ = false;
}

void AutoNavHandler::cmdVelNavCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    lx_msgs::msg::RoverCommand rov_cmd;
    rov_cmd.mobility_twist = *msg;
    
    // Publish rover command
    rover_cmd_pub_->publish(rov_cmd);
}