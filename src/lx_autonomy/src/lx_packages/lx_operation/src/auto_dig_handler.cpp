/* Author: Vibhakar Mohta and Dhruv Tyagi
 * Subscribers:
 *    - /tool_info: [lx_msgs::msg::ToolInfo] The info from tool sensors
 *    - /tool_height: [std_msgs::msg::Float64] The height of the tool
 * Publishers:
 *    - /rover_auto_cmd: [lx_msgs::msg::RoverCommand] The autonomy command to the rover
 *    - /lx_diagnostics: [lx_msgs::msg::NodeDiagnostics] The diagnostic heartbeat
 * Services:
 *    - /lx_param_server_node/get_parameters: [rcl_interfaces::srv::GetParameters] Get parameters from the global parameter server
 * Actions:
 *    - /operations/autodig_action: [lx_msgs::action::AutoDig] The action server for autodig
 *
 * - Handles the AutoDig action request. Commands the rover to excavate in a straight line while excavating, trying to 
 * track the desired current value. 2 Loops of PID control are used, the outer loop controls the desired tool height 
 * while the inner loop tracks the desired tool height.
 * 
 * */

#include "lx_operation/auto_dig_handler.hpp"

AutoDigHandler::AutoDigHandler(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()): Node("auto_dig_handler_node"){
    (void)options;

    // Set up subscriptions, publishers, services, action servers and clients
    setupCommunications();

    // Timers
    rover_command_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), 
                        std::bind(&AutoDigHandler::roverCommandTimerCallback, this));
    diagnostic_pub_timer_ = this->create_wall_timer(std::chrono::seconds(diagnostic_pub_period_), 
                        std::bind(&AutoDigHandler::diagnosticPublish, this));

    // Set PID values
    autodig_pid_outer_.kp = 0.013;
    autodig_pid_outer_.ki = 0.000001;
    autodig_pid_outer_.kd = 0.02;

    autodig_pid_inner_.kp = 20.0;
    autodig_pid_inner_.ki = 0.0001;
    autodig_pid_inner_.kd = 0.5;

    // Get parameters from the global parameter server
    getParams();

    // Set up parameters from the global parameter server
    setupParams();


    RCLCPP_INFO(this->get_logger(), "AutoDig handler initialized");
}

void AutoDigHandler::getParams(){
    while(!get_params_client_->wait_for_service(std::chrono::seconds(2))){
      RCLCPP_INFO(this->get_logger(), "Could not contact param server");
      return;
    }
    // Get important parameters
    auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    get_request->names = {"rover.mobility_lock", "rover.actuation_lock", 
                          "rover.op_mode", "rover.task_mode"};
    // Send request
    auto param_result_ = get_params_client_->async_send_request(get_request,std::bind(&AutoDigHandler::paramCB, this, std::placeholders::_1));
}

void AutoDigHandler::paramCB(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future){
    RCLCPP_INFO(this->get_logger(), "1");
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

void AutoDigHandler::setupCommunications(){
    // Subscribers 
    tool_info_sub_ = this->create_subscription<lx_msgs::msg::ToolInfo>("/tool_info", 10, 
                        std::bind(&AutoDigHandler::toolInfoCB, this, std::placeholders::_1));
    drum_height_sub_ = this->create_subscription<std_msgs::msg::Float64>("/tool_height", 10, 
                        std::bind(&AutoDigHandler::drumHeightCB, this, std::placeholders::_1));
    
    // Publishers
    rover_auto_cmd_pub_ = this->create_publisher<lx_msgs::msg::RoverCommand>("/rover_auto_cmd", 10);
    diagnostic_publisher_ = this->create_publisher<lx_msgs::msg::NodeDiagnostics>("lx_diagnostics", 10);
    if(debugging_publish_){ 
        drum_desired_current_pub_ = this->create_publisher<std_msgs::msg::Float64>("/drum_desired_current", 10);
        drum_current_current_pub_ = this->create_publisher<std_msgs::msg::Float64>("/drum_current_current", 10);
        drum_desired_height_pub_ = this->create_publisher<std_msgs::msg::Float64>("/drum_desired_height", 10);
        drum_current_height_pub_ = this->create_publisher<std_msgs::msg::Float64>("/drum_current_height", 10);
    }

    // Service clients
    get_params_client_ = this->create_client<rcl_interfaces::srv::GetParameters>("/lx_param_server_node/get_parameters");

    // Action server
    using namespace std::placeholders;
    this->autodig_action_server_ = rclcpp_action::create_server<AutoDig>(this, "operations/autodig_action",
                                            std::bind(&AutoDigHandler::handle_goal, this, _1, _2),
                                            std::bind(&AutoDigHandler::handle_cancel, this, _1),
                                            std::bind(&AutoDigHandler::handle_accepted, this, _1));
}

void AutoDigHandler::setupParams(){
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

rclcpp_action::GoalResponse AutoDigHandler::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const AutoDig::Goal> goal){
    RCLCPP_INFO(this->get_logger(), "Received autodig request");
    (void)uuid;
    (void)goal;
    
    // Accept and execute action
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AutoDigHandler::handle_cancel(const std::shared_ptr<GoalHandleAutoDig> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;

    // Set the rover free from inner PID control
    inner_PID_control_rover_ = false;
    
    // Cancel action
    return rclcpp_action::CancelResponse::ACCEPT;
}

void AutoDigHandler::handle_accepted(const std::shared_ptr<GoalHandleAutoDig> goal_handle){
    using namespace std::placeholders;
    
    // Start thread to execute action and detach
    std::thread{std::bind(&AutoDigHandler::executeAutoDig, this, _1), goal_handle}.detach();
}

void AutoDigHandler::executeAutoDig(const std::shared_ptr<GoalHandleAutoDig> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Executing autodig request");
    
    // Get goal, feedback and result
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<AutoDig::Feedback>();
    auto result = std::make_shared<AutoDig::Result>();

    // Reset integral errors
    integral_error_current = 0;
    integral_error_height = 0;

    // Allow rover to be controlled by inner PID loop
    inner_PID_control_rover_ = true; 

    rclcpp::Time action_start_time = this->get_clock()->now();

    // Wait for drum height to reach GOTO_TOOL_HEIGHT
    target_drum_height = GOTO_TOOL_HEIGHT;
    while(rclcpp::ok() && !goal_handle->is_canceling()){
        if(std::abs(drum_height_-target_drum_height) < 0.02){
            break;
        }
    }

    // 10 Hz loop
    rclcpp::Rate loop_rate(10);

    while(rclcpp::ok() && !goal_handle->is_canceling()){
        rclcpp::Time action_curr_time = this->get_clock()->now();

        // Abort action if no tool info message recieved in last 1 second
        if((action_curr_time - tool_info_msg_time_).seconds() > 1){
            // Set the rover free from inner PID control
            inner_PID_control_rover_ = false;
            result->success = false;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Autodig failed due to no tool info message timeout");
            return;
        }

        // End action after T_END_SECONDS seconds
        double action_time_diff_seconds = (action_curr_time - action_start_time).seconds();
        if(action_time_diff_seconds > T_END_SECONDS){
            break; 
        }
        
        double desired_current_value = NOMINAL_CURRENT_VALUE_I + 
                                        (NOMINAL_CURRENT_VALUE_F - NOMINAL_CURRENT_VALUE_I) * action_time_diff_seconds / (T_END_SECONDS);

        if(debugging_publish_){
            // Publish drum desired current and drum current current
            std_msgs::msg::Float64 msg; 
            msg.data = desired_current_value;
            drum_desired_current_pub_->publish(msg);
            msg.data = tool_info_msg_.drum_current;
            drum_current_current_pub_->publish(msg);
        }
        
        // PID Outer Loop
        double drum_current_error = tool_info_msg_.drum_current - desired_current_value;
        double drum_current_error_pid = autodig_pid_outer_.kp*drum_current_error 
                                        + autodig_pid_outer_.ki*integral_error_current 
                                        + autodig_pid_outer_.kd*(drum_current_error - prev_error_current);
        prev_error_current = drum_current_error;
        integral_error_current += drum_current_error;

        // Set targets for inner loop
        target_drum_command = DRUM_COMMAND_EXCAVATION;
        target_rover_velocity = FORWARD_SPEED;
        target_drum_height = drum_height_ + drum_current_error_pid;

        // Clip target_drum_height
        target_drum_height = std::min(std::max(target_drum_height, OUTER_PID_CLIP_MIN), OUTER_PID_CLIP_MAX);
        
        RCLCPP_DEBUG(this->get_logger(), "[OUTER LOOP] Error: %.3f, Diff: %.3f, Target_Height: %.3f ", 
                                        drum_current_error, drum_current_error_pid, target_drum_height);

        loop_rate.sleep();
    }

    // Stop drum and rover movement
    target_rover_velocity = 0;
    target_drum_command = 0;
    integral_error_current = 0;
    integral_error_height = 0;

    // Raise the drum to END_TOOL_HEIGHT
    target_drum_height = END_TOOL_HEIGHT;
    while(rclcpp::ok() && !goal_handle->is_canceling()){
        if(std::abs(drum_height_-target_drum_height) < 0.02){
            break;
        }
    }
    // Set targets to 0 to stop the rover
    target_drum_height = -1;
    integral_error_current = 0;
    integral_error_height = 0;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    inner_PID_control_rover_ = false;
    
    // If autodig executed successfully, return goal success
    if (rclcpp::ok() && !goal_handle->is_canceling()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Autodig succeeded");
    }
    else if(rclcpp::ok() && goal_handle->is_canceling()){
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Autodig canceled");
    }
    else{
        result->success = false;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Autodig failed");
    }
}

void AutoDigHandler::toolInfoCB(const lx_msgs::msg::ToolInfo::SharedPtr msg){
    tool_info_msg_ = *msg;
    tool_info_msg_time_ = this->get_clock()->now();
}

void AutoDigHandler::drumHeightCB(const std_msgs::msg::Float64::SharedPtr msg){
    drum_height_ = msg->data;
}

void AutoDigHandler::roverCommandTimerCallback(){

    // if control_rover_ is false, do nothing
    if(this->inner_PID_control_rover_==false){
        return;
    } 

    lx_msgs::msg::RoverCommand rover_cmd;

    // Stop everything if target_drum_height is -1
    if(target_drum_height == -1){
        rover_cmd.actuator_speed = 0;
        rover_cmd.drum_speed = 0;
        rover_cmd.mobility_twist.linear.x = 0;
    }
    else{
        double error_height = drum_height_ - target_drum_height;
        double pid_command = autodig_pid_inner_.kp*error_height 
                            + autodig_pid_inner_.ki*integral_error_height 
                            + autodig_pid_inner_.kd*(error_height - prev_error_height);

        if(std::abs(error_height) < 0.01){
            pid_command = 0;
        }

        RCLCPP_DEBUG(this->get_logger(), "[INNER LOOP] Error: %.3f, PID Command: %.3f", error_height, pid_command);

        // Store rover command
        rover_cmd.actuator_speed = std::min(std::max(pid_command, -1.0), 1.0); //clip drum_height_error_pid to [-1, 1]
        rover_cmd.drum_speed = target_drum_command;
        rover_cmd.mobility_twist.linear.x = target_rover_velocity;

        if(debugging_publish_){
            // Publish using debug publishers
            std_msgs::msg::Float64 msg; 
            msg.data = target_drum_height;
            this->drum_desired_height_pub_->publish(msg);
            msg.data = drum_height_;
            this->drum_current_height_pub_->publish(msg);
        }

        integral_error_height += error_height;
        prev_error_height = error_height;
    }
    rover_auto_cmd_pub_->publish(rover_cmd);
}

void AutoDigHandler::diagnosticPublish(){
    // Publish diagnostic message
    auto msg = lx_msgs::msg::NodeDiagnostics();
    msg.node_name = this->get_name();
    msg.stamp = this->get_clock()->now();
    diagnostic_publisher_->publish(msg);
}