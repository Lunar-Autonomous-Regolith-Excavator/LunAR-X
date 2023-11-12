/* Author: Vibhakar Mohta
 * Subscribers:
 *    - /topic: description
 * Publishers:
 *    - /topic: description
 * Services:
 *    - /name (type): description
 *
 * - Auto Dump Action Server
 * 
 * TODO
 * - Angular berm dump
 * */

#include "lx_operation/auto_dump_handler.hpp"


AutoDumpHandler::AutoDumpHandler(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()): Node("auto_dump_handler_node"){
    (void)options;

    // Set up subscriptions, publishers, services, action servers and clients
    setupCommunications();

    // Get parameters from the global parameter server
    getParams();

    // Set up parameters from the global parameter server
    setupParams();

    // Set PID values
    tool_height_pid_.kp = 8;
    tool_height_pid_.ki = 0.0002;
    tool_height_pid_.kd = 0.5;

    rover_x_pid_.kp = 1.2;
    rover_x_pid_.ki = 0.0001;
    rover_x_pid_.kd = 0.05;

    rover_yaw_pid_.kp = 8;
    rover_yaw_pid_.ki = 0.001;
    rover_yaw_pid_.kd = 0.6;

    RCLCPP_INFO(this->get_logger(), "AutoDump handler initialized");
}

void AutoDumpHandler::getParams(){
    while(!get_params_client_->wait_for_service(std::chrono::seconds(2))){
      RCLCPP_INFO(this->get_logger(), "Could not contact param server");
      return;
    }
    // Get important parameters
    auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    get_request->names = {"rover.mobility_lock", "rover.actuation_lock", 
                          "rover.op_mode", "rover.task_mode"};
    // Send request
    auto param_result_ = get_params_client_->async_send_request(get_request,std::bind(&AutoDumpHandler::paramCB, this, std::placeholders::_1));
}

void AutoDumpHandler::paramCB(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future){
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

void AutoDumpHandler::setupCommunications(){
    // Subscribers 
    drum_height_sub_ = this->create_subscription<std_msgs::msg::Float64>("/tool_height", 1,
                        std::bind(&AutoDumpHandler::drumHeightCB, this, std::placeholders::_1));
    visual_servo_error_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/mapping/visual_servo_error", 1,
                        std::bind(&AutoDumpHandler::visualServoErrorCB, this, std::placeholders::_1));

    // Publishers
    rover_auto_cmd_pub_ = this->create_publisher<lx_msgs::msg::RoverCommand>("/rover_auto_cmd", 1);

    // Service clients
    get_params_client_ = this->create_client<rcl_interfaces::srv::GetParameters>("/lx_param_server_node/get_parameters");
    visual_servo_client_ = this->create_client<lx_msgs::srv::Switch>("/mapping/visual_servo_switch");

    // Action server
    using namespace std::placeholders;
    this->autodump_action_server_ = rclcpp_action::create_server<AutoDump>(this, "operations/autodump_action",
                                            std::bind(&AutoDumpHandler::handle_goal, this, _1, _2),
                                            std::bind(&AutoDumpHandler::handle_cancel, this, _1),
                                            std::bind(&AutoDumpHandler::handle_accepted, this, _1));
}

void AutoDumpHandler::setupParams(){
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

rclcpp_action::GoalResponse AutoDumpHandler::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const AutoDump::Goal> goal){
    RCLCPP_INFO(this->get_logger(), "Received autodump request");
    (void)uuid;
    (void)goal;
    
    // Accept and execute action
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AutoDumpHandler::handle_cancel(const std::shared_ptr<GoalHandleAutoDump> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    
    // Cancel action
    return rclcpp_action::CancelResponse::ACCEPT;
}

void AutoDumpHandler::handle_accepted(const std::shared_ptr<GoalHandleAutoDump> goal_handle){
    using namespace std::placeholders;
    
    // Start thread to execute action and detach
    std::thread{std::bind(&AutoDumpHandler::executeAutoDump, this, _1), goal_handle}.detach();
}

void AutoDumpHandler::executeAutoDump(const std::shared_ptr<GoalHandleAutoDump> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Executing autodump request");
    
    // Get goal, feedback and result
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<AutoDump::Feedback>();
    auto result = std::make_shared<AutoDump::Result>();

    // Move to targets
    // 3 PIDs for drum height, rover x and rover y
    auto pid_height = PID(tool_height_pid_, -CLIP_HEIGHT_CMD_VAL, CLIP_HEIGHT_CMD_VAL);
    auto pid_x = PID(rover_x_pid_, -CLIP_VEL_CMD_VAL, CLIP_VEL_CMD_VAL);
    auto pid_yaw = PID(rover_yaw_pid_, -CLIP_YAW_CMD_VAL, CLIP_YAW_CMD_VAL);

    // Start Visual Servoing if goal->first_op_dump if false
    if (goal->first_op_dump == false)
    {
        // Start Visual Servoing
        if(!callVisualServoSwitch(true)){
            result->success = false;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Autodump failed due to visual servoing switch on failure");
            return;
        }
    }

    lx_msgs::msg::RoverCommand rover_cmd;
    rclcpp::Rate loop_rate(10);
    while(rclcpp::ok() && !goal_handle->is_canceling())
    {
        RCLCPP_INFO(this->get_logger(), "[AUTODUMP] Visual servoing");
        rclcpp::Time action_curr_time = this->get_clock()->now();

        // Abort action if no tool info message recieved in last 2 seconds
        if((action_curr_time - servoing_msg_time).seconds() > 2 && goal->first_op_dump == false){
            result->success = false;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Autodig failed due to no tool info message timeout");
            // Stop visual servoing
            if(!callVisualServoSwitch(false)){
                result->success = false;
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "Autodump failed due to visual servoing switch off failure");
            }
            return;
        }
        // Read errors from visual servoing
        double dx  = visual_servo_error_.x;
        double dy = -visual_servo_error_.y;
        double dz = -visual_servo_error_.z;

        // Skip x and y control if first op dump
        if(goal->first_op_dump == false)
        {
            dx = 0;
            dy = 0;
            dz = drum_height_ - 0.3;
        }
        double drum_cmd, x_vel, yaw_vel;
        if(abs(dx) < 0.02){x_vel = 0;}
        else{x_vel = pid_x.getCommand(dx);}

        if(abs(dy) < 0.03){yaw_vel = 0;}
        else{yaw_vel = pid_yaw.getCommand(dy);}

        if (abs(dz) < 0.02) {drum_cmd = 0;}
        else{drum_cmd = pid_height.getCommand(dz);}

        // Publish to rover
        rover_cmd.actuator_speed =  drum_cmd;
        rover_cmd.mobility_twist.linear.x = x_vel;
        rover_cmd.mobility_twist.angular.z = yaw_vel;
        this->rover_auto_cmd_pub_->publish(rover_cmd);
        
        RCLCPP_INFO(this->get_logger(), "[AUTODUMP] Drum: Error: %f, Command: %f", dz, drum_cmd);
        RCLCPP_INFO(this->get_logger(), "[AUTODUMP] Rover x: Error: %f, Command: %f", dx, x_vel);
        RCLCPP_INFO(this->get_logger(), "[AUTODUMP] Rover y: Error: %f, Command: %f", dy, yaw_vel);

        if (abs(dz) < 0.02 && abs(dx) < 0.02 && abs(dy) < 0.03)
        {
            RCLCPP_INFO(this->get_logger(), "Autodump reached targets");
            break;
        }

        loop_rate.sleep();
    }
    // Set targets to 0 to stop the rover
    rover_cmd.actuator_speed = 0;
    rover_cmd.mobility_twist.linear.x = 0;
    this->rover_auto_cmd_pub_->publish(rover_cmd);

    // Stop Visual Servoing
    if(!callVisualServoSwitch(false)){
        result->success = false;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Autodump failed due to visual servoing switch off failure");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Finished visual servoing");

    // Dump!
    auto dump_start_time = this->get_clock()->now();
    while (rclcpp::ok() && !goal_handle->is_canceling())
    {   
        // get time in ms
        auto time_diff_s = (this->get_clock()->now() - dump_start_time).seconds();
        if(time_diff_s > DRUM_DUMP_TIME_S){
            RCLCPP_INFO(this->get_logger(), "Autodump finished dumping");
            break;
        }
        rover_cmd.drum_speed = DRUM_DUMP_SPEED;
        this->rover_auto_cmd_pub_->publish(rover_cmd);
        RCLCPP_INFO(this->get_logger(), "[AUTODUMP] Dumping material");
        loop_rate.sleep();
    }
    // Set targets to 0 to stop the rover
    rover_cmd.drum_speed = 0;
    rover_cmd.actuator_speed = 0;
    rover_cmd.mobility_twist.linear.x = 0;
    this->rover_auto_cmd_pub_->publish(rover_cmd);

    RCLCPP_INFO(this->get_logger(), "Finished dumping");

    // Raise the drum to END_TOOL_HEIGHT
    pid_height = PID(tool_height_pid_, -CLIP_HEIGHT_CMD_VAL, CLIP_HEIGHT_CMD_VAL);
    while(rclcpp::ok() && !goal_handle->is_canceling())
    {
        double drum_height_error = drum_height_ - END_TOOL_HEIGHT;
        double drum_pid_command = -0.8;

        // Publish to rover
        rover_cmd.actuator_speed =  drum_pid_command;
        rover_cmd.mobility_twist.linear.x = 0;
        this->rover_auto_cmd_pub_->publish(rover_cmd);
        
        RCLCPP_INFO(this->get_logger(), "[AUTODUMP] Drum height: %f, Target: %f, Error: %f, Command: %f", drum_height_, END_TOOL_HEIGHT, drum_height_error, drum_pid_command);
        loop_rate.sleep();

        if (abs(drum_height_error) < 0.01)
        {
            RCLCPP_INFO(this->get_logger(), "Autodump raised drum to target");
            break;
        }
    }
    // Set targets to 0 to stop the rover
    rover_cmd.actuator_speed = 0;
    rover_cmd.mobility_twist.linear.x = 0;
    this->rover_auto_cmd_pub_->publish(rover_cmd);
    
    // If autodump executed successfully, return goal success
    if (rclcpp::ok() && !goal_handle->is_canceling()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Autodump succeeded");
    }
    else if(rclcpp::ok() && goal_handle->is_canceling()){
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Autodump canceled");
    }
    else{
        result->success = false;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Autodump failed");
    }
}

void AutoDumpHandler::drumHeightCB(const std_msgs::msg::Float64::SharedPtr msg){
    this->drum_height_ = msg->data;
}

void AutoDumpHandler::visualServoErrorCB(const geometry_msgs::msg::Point::SharedPtr msg){
    this->visual_servo_error_ = *msg;
    this->servoing_msg_time = this->get_clock()->now();
}

bool AutoDumpHandler::callVisualServoSwitch(bool request_switch_state){
    auto switch_request = std::make_shared<lx_msgs::srv::Switch::Request>();
    switch_request->switch_state = request_switch_state;
    while(!visual_servo_client_->wait_for_service(std::chrono::seconds(2))){
        RCLCPP_INFO(this->get_logger(), "Could not contact visual servo switch");
        return false;
    }
    visual_servo_switch_ = false;
    
    // Send request and wait for response
    auto visual_servo_result = visual_servo_client_->async_send_request(switch_request, std::bind(&AutoDumpHandler::visualServoSwitchCB, 
                                                                                                    this, std::placeholders::_1));
    
    // Wait for visual servoing to start, wait for 3 seconds
    auto start_time = this->get_clock()->now();
    while(!visual_servo_switch_){
        RCLCPP_WARN(this->get_logger(), "Waiting for visual servoing to start");
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        if((this->get_clock()->now() - start_time).seconds() > 3){
            RCLCPP_ERROR(this->get_logger(), "Visual servoing failed to start");
            return false;
        }
    }
    return true;
}

void AutoDumpHandler::visualServoSwitchCB(rclcpp::Client<lx_msgs::srv::Switch>::SharedFuture future){
    auto status = future.wait_for(std::chrono::milliseconds(100));
    if (status == std::future_status::ready && future.get()->success){
        RCLCPP_INFO(this->get_logger(), "Visual servoing started");
        visual_servo_switch_ = true;
    } 
    else {
        RCLCPP_INFO(this->get_logger(), "Visual servoing failed to start");
    }
}