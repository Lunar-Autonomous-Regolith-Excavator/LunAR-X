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

#include "lx_operation/auto_dig_handler.hpp"

AutoDigHandler::AutoDigHandler(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()): Node("auto_dig_handler_node"){
    (void)options;
    // Initialize timers
    tool_info_msg_time_ = this->get_clock()->now();

    // Set up subscriptions, publishers, services, action servers and clients
    setupCommunications();

    // Set PID to 0.0 for safety until parameters are updated
    autodig_pid_.kp = 0.0;
    autodig_pid_.ki = 0.0;
    autodig_pid_.kd = 0.0;

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
                          "rover.op_mode", "rover.task_mode",
                          "autodig.pid"};
    // Send request
    auto param_result_ = get_params_client_->async_send_request(get_request,std::bind(&AutoDigHandler::paramCB, this, std::placeholders::_1));
}

void AutoDigHandler::paramCB(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future){
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
        autodig_pid_.kp = future.get()->values.at(4).double_array_value[0];
        autodig_pid_.ki = future.get()->values.at(4).double_array_value[1];
        autodig_pid_.kd = future.get()->values.at(4).double_array_value[2];
        RCLCPP_INFO(this->get_logger(), "Parameter set Autodig PID: [%.3f, %.5f, %.3f]", autodig_pid_.kp, autodig_pid_.ki, autodig_pid_.kd);
    } 
    else {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}

void AutoDigHandler::toolInfoCB(const lx_msgs::msg::ToolInfo::SharedPtr msg){
    tool_info_msg_ = *msg;
    tool_info_msg_time_ = this->get_clock()->now();
}

void AutoDigHandler::setupCommunications(){
    // Subscribers 
    tool_info_sub_ = this->create_subscription<lx_msgs::msg::ToolInfo>("/tool_info", 10, 
                        std::bind(&AutoDigHandler::toolInfoCB, this, std::placeholders::_1));
    
    // Publishers
    rover_hw_cmd_pub_ = this->create_publisher<lx_msgs::msg::RoverCommand>("/rover_auto_cmd", 10);
    drum_desired_current_pub_ = this->create_publisher<std_msgs::msg::Float64>("/drum_desired_current", 10);
    drum_current_current_pub_ = this->create_publisher<std_msgs::msg::Float64>("/drum_current_current", 10);
    
    // Service servers

    // Service clients
    get_params_client_ = this->create_client<rcl_interfaces::srv::GetParameters>("/param_server_node/get_parameters");
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
    auto autodig_pid_params_callback = [this](const rclcpp::Parameter & p){
        autodig_pid_.kp = p.as_double_array()[0];
        autodig_pid_.ki = p.as_double_array()[1];
        autodig_pid_.kd = p.as_double_array()[2];
        RCLCPP_INFO(this->get_logger(), "Parameter updated \"%s\": [%.3f, %.5f, %.3f]", p.get_name().c_str(), autodig_pid_.kp, autodig_pid_.ki, autodig_pid_.kd);
    };

    // Names of node & params for adding callback
    auto param_server_name = std::string("param_server_node");
    auto mob_lock_param_name = std::string("rover.mobility_lock");
    auto act_lock_param_name = std::string("rover.actuation_lock");
    auto op_mode_param_name = std::string("rover.op_mode");
    auto task_mode_param_name = std::string("rover.task_mode");
    auto autodig_pid_param_name = std::string("autodig.pid");

    // Store callback handles for each parameter
    mob_param_cb_handle_ = param_subscriber_->add_parameter_callback(mob_lock_param_name, mob_params_callback, param_server_name);
    act_param_cb_handle_ = param_subscriber_->add_parameter_callback(act_lock_param_name, act_params_callback, param_server_name);
    op_mode_param_cb_handle_ = param_subscriber_->add_parameter_callback(op_mode_param_name, op_mode_params_callback, param_server_name);
    task_mode_param_cb_handle_ = param_subscriber_->add_parameter_callback(task_mode_param_name, task_mode_params_callback, param_server_name);
    autodig_pid_param_cb_handle_ = param_subscriber_->add_parameter_callback(autodig_pid_param_name, autodig_pid_params_callback, param_server_name);
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

    // If last message was recieved more than 1 second ago, return goal aborted
    if((this->get_clock()->now() - tool_info_msg_time_).seconds() > 1){
        result->success = false;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Autodig failed due to no tool info message timeout");
        return;
    }
    
    rclcpp::Time action_start_time = this->get_clock()->now();
    rclcpp::Rate loop_rate_1(0.01);
    while(true)
    {
        rclcpp::Time action_curr_time = this->get_clock()->now();
        double action_time_diff_seconds = (action_curr_time - action_start_time).seconds();
        if(action_time_diff_seconds > t_end_seconds)
            break;
        
        double desired_current_value = nominal_current_value_i + (nominal_current_value_f - nominal_current_value_i) * action_time_diff_seconds / (t_end_seconds);
        // Publish drum desired current and drum current current
        std_msgs::msg::Float64 drum_desired_current_msg;
        drum_desired_current_msg.data = desired_current_value;
        drum_desired_current_pub_->publish(drum_desired_current_msg);

        std_msgs::msg::Float64 drum_current_current_msg;
        drum_current_current_msg.data = tool_info_msg_.drum_current;
        drum_current_current_pub_->publish(drum_current_current_msg);

        // PID
        double drum_current_error = tool_info_msg_.drum_current - desired_current_value;
        
        double drum_current_error_pid = autodig_pid_.kp*drum_current_error + autodig_pid_.ki*integral_error + autodig_pid_.kd*(drum_current_error - prev_error);
        prev_error = drum_current_error;
        integral_error += drum_current_error;

        RCLCPP_INFO(this->get_logger(), "Drum current error: %f", drum_current_error);

        // if the value of pid_drum_height is above pid_height_error_threshold_cm, control drum for height_control_period_seconds
        if(abs(drum_current_error_pid) > drum_control_error_thresh)
        {
            rclcpp::Time start = this->get_clock()->now(), curr_time = this->get_clock()->now();
            rclcpp::Rate loop_rate_2(0.1);
            while(true)
            {
                curr_time = this->get_clock()->now();
                double time_diff_seconds = (curr_time - start).seconds();
                if(time_diff_seconds > height_control_period_seconds)
                    break;

                // Set drum command
                lx_msgs::msg::RoverCommand rover_cmd;
                // set it based on sign of pid_drum_height
                rover_cmd.actuator_speed = drum_current_error_pid > 0 ? -actuator_control_value : actuator_control_value;
                rover_cmd.drum_speed = drum_command;
                rover_cmd.mobility_twist.linear.x = forward_speed;
                rover_hw_cmd_pub_->publish(rover_cmd);
                RCLCPP_INFO(this->get_logger(), "Actuating Linear Actuator with value: %f", rover_cmd.actuator_speed);
                // std::this_thread::sleep_for(std::chrono::milliseconds(100));
                loop_rate_2.sleep();
            }
        }
        else
        {
            // keep going forward
            lx_msgs::msg::RoverCommand rover_cmd;
            rover_cmd.mobility_twist.linear.x = forward_speed;
            rover_cmd.drum_speed = drum_command;
            rover_hw_cmd_pub_->publish(rover_cmd);
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        loop_rate_1.sleep();
    }

    // publish a 0 rover command to stop the rover
    lx_msgs::msg::RoverCommand rover_cmd;
    rover_cmd.mobility_twist.linear.x = 0;
    rover_cmd.drum_speed = 0;
    rover_cmd.actuator_speed = 0;
    rover_hw_cmd_pub_->publish(rover_cmd);
    

    // If autodig fails, return goal aborted. Uncomment following:

    // result->success = false;
    // goal_handle->abort(result);
    // RCLCPP_ERROR(this->get_logger(), "Autodig failed");
    // return;
    
    // If autodig executed successfully, return goal success
    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Autodig succeeded");
    }
}