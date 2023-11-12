/* Author: Dhruv Tyagi
 * Subscribers:
 *    - /joy: [sensor_msgs::msg::Joy] Joystick raw input
 * Publishers:
 *    - /rover_teleop_cmd: [lx_msgs::msg::RoverCommand] Teleop command passthrough to command_mux_node
 * Services:
 *    - /lx_param_server_node/set_parameters - Client - [rcl_interfaces::srv::SetParameters] Client to set or change global params on lx_param_server_node
 *
 * - Based on user inputs, guides the robot operation
 * - Joystick buttons set the lock, operation mode and task mode
 * - If the op mode is set to teleop, will passthrough the joystick teleop commands to the command mux via RoverCommand data type
 * - If no joystick data received for 3 seconds, will set the rover to lock for safety.
 * 
 * TODO
 * - Check start and stop mapping service with joystick and lx_mapping
 * */

#include "lx_external_interface/external_interface.hpp"

ExternalInterface::ExternalInterface(): Node("external_interface_node"){
    // Lock movement at system start
    rover_soft_lock_.mobility_lock = true;
    rover_soft_lock_.actuation_lock = true;

    // Timer for active rover lock
    rover_lock_timer_ = this->create_wall_timer(std::chrono::seconds(3), 
                        std::bind(&ExternalInterface::activeLock, this));
    // Timer for diagnostics publisher
    diagnostic_pub_timer_ = this->create_wall_timer(std::chrono::seconds(diagnostic_pub_period_), 
                        std::bind(&ExternalInterface::diagnosticPublish, this));

    // Timers for debouncing
    guide_debounce_timer_ = this->get_clock()->now();
    start_debounce_timer_ = this->get_clock()->now();
    back_debounce_timer_ = this->get_clock()->now();
    a_debounce_timer_ = this->get_clock()->now();
    b_debounce_timer_ = this->get_clock()->now();
    y_debounce_timer_ = this->get_clock()->now();
    
    // Set up subscriptions & publishers
    setupCommunications();

    // Get parameters from the global parameter server
    getParams();

    // Set up parameters from the global parameter server
    setupParams();

    // Publish rover lock
    switchRoverLockStatus(true, true);

    // Publish op_mode : standby
    switchRoverOpMode(OpModeEnum::STANDBY);

    // Publish task_mode : idle
    switchRoverTaskMode(TaskModeEnum::IDLE);

    RCLCPP_INFO(this->get_logger(), "External Interface initialized");
}

void ExternalInterface::setupCommunications(){
    // Subscribers
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10 , 
                            std::bind(&ExternalInterface::joyCallBack, this, std::placeholders::_1));

    // Publishers
    rover_teleop_publisher_ = this->create_publisher<lx_msgs::msg::RoverCommand>("rover_teleop_cmd", 10);
    diagnostic_publisher_ = this->create_publisher<lx_msgs::msg::NodeDiagnostics>("lx_diagnostics", 10);

    // Clients
    set_params_client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/lx_param_server_node/set_parameters");
    map_switch_client_ = this->create_client<lx_msgs::srv::Switch>("/mapping/map_switch");
    get_params_client_ = this->create_client<rcl_interfaces::srv::GetParameters>("/lx_param_server_node/get_parameters");
    calibrate_imu_action_client_ = rclcpp_action::create_client<CalibrateImu>(this, "/lx_localization/calibrate_imu");
}

void ExternalInterface::lockCheck(){
    // Check if the lock statuses have flipped
    if(rover_soft_lock_.mobility_lock != rover_soft_lock_.actuation_lock){
        // If so, lock both
        switchRoverLockStatus(true, true);
        RCLCPP_WARN(this->get_logger(), "Mobility and actuation locks are not in sync, locking both");
    }
}

void ExternalInterface::getParams(){
    while(!get_params_client_->wait_for_service(std::chrono::seconds(2))){
      RCLCPP_INFO(this->get_logger(), "Could not contact param server");
      return;
    }

    // Get important parameters
    auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    get_request->names = {"rover.mobility_lock", "rover.actuation_lock", 
                          "rover.op_mode", "rover.task_mode", 
                          "limits.max_lin_mob_vel", "limits.max_ang_mob_vel"};

    // Send request
    auto param_result_ = get_params_client_->async_send_request(get_request,std::bind(&ExternalInterface::paramCB, this, std::placeholders::_1));
}

void ExternalInterface::paramCB(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future){
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

        mob_lin_vel_ = future.get()->values.at(4).double_value;
        RCLCPP_DEBUG(this->get_logger(), "Parameter set Max Linear Vel: %.2f", mob_lin_vel_);
        mob_ang_vel_ = future.get()->values.at(5).double_value;
        RCLCPP_DEBUG(this->get_logger(), "Parameter set Max Angular Vel: %.2f", mob_ang_vel_);
    } 
    else {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}

void ExternalInterface::setupParams(){
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
    auto lin_mob_vel_params_callback = [this](const rclcpp::Parameter & p){
        RCLCPP_DEBUG(this->get_logger(), "Parameter updated \"%s\": %.2f", p.get_name().c_str(), p.as_double());
        mob_lin_vel_ = p.as_double();
    };
    auto ang_mob_vel_params_callback = [this](const rclcpp::Parameter & p){
        RCLCPP_DEBUG(this->get_logger(), "Parameter updated \"%s\": %.2f", p.get_name().c_str(), p.as_double());
        mob_ang_vel_ = p.as_double();
    };

    // Names of node & params for adding callback
    auto param_server_name = std::string("lx_param_server_node");
    auto mob_lock_param_name = std::string("rover.mobility_lock");
    auto act_lock_param_name = std::string("rover.actuation_lock");
    auto op_mode_param_name = std::string("rover.op_mode");
    auto task_mode_param_name = std::string("rover.task_mode");
    auto lin_mob_vel_param_name = std::string("operational.max_lin_mob_vel");
    auto ang_mob_vel_param_name = std::string("operational.max_ang_mob_vel");

    // Store callback handles for each parameter
    mob_param_cb_handle_ = param_subscriber_->add_parameter_callback(mob_lock_param_name, mob_params_callback, param_server_name);
    act_param_cb_handle_ = param_subscriber_->add_parameter_callback(act_lock_param_name, act_params_callback, param_server_name);
    op_mode_param_cb_handle_ = param_subscriber_->add_parameter_callback(op_mode_param_name, op_mode_params_callback, param_server_name);
    task_mode_param_cb_handle_ = param_subscriber_->add_parameter_callback(task_mode_param_name, task_mode_params_callback, param_server_name);
    lin_mob_vel_param_cb_handle_ = param_subscriber_->add_parameter_callback(lin_mob_vel_param_name, lin_mob_vel_params_callback, param_server_name);
    ang_mob_vel_param_cb_handle_ = param_subscriber_->add_parameter_callback(ang_mob_vel_param_name, ang_mob_vel_params_callback, param_server_name);
}

void ExternalInterface::joyCallBack(const sensor_msgs::msg::Joy::SharedPtr joy_msg){

    // Publisher given information to publish rover-op-mode and teleop commands 
    rover_control_pub_thread_ = std::thread(std::bind(&ExternalInterface::roverControlPublish, this, joy_msg));

    // Have to detach thread before it goes out of scope
    rover_control_pub_thread_.detach(); 

}

void ExternalInterface::roverControlPublish(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
    // Active lock timer reset
    rover_lock_timer_->reset();

    // Guide-button rising-edge controls locking of actuation & mobility 
    if(joy_msg->buttons[int(JoyButtons::GUIDE)] && !joy_last_state_.buttons[int(JoyButtons::GUIDE)]){
        // Check debounce time
        if((this->get_clock()->now() - guide_debounce_timer_).seconds() > 0.1){
            // Change lock status
            switchRoverLockStatus(!rover_soft_lock_.mobility_lock,!rover_soft_lock_.actuation_lock);
            guide_debounce_timer_ = this->get_clock()->now();
        }
    }

    // Start-button rising-edge cycles through the operating modes of the rover
    if(joy_msg->buttons[int(JoyButtons::START)] && !joy_last_state_.buttons[int(JoyButtons::START)]){
        // Check debounce time
        if((this->get_clock()->now() - start_debounce_timer_).seconds() > 0.1){
            switch(current_rover_op_mode_){
                case OpModeEnum::STANDBY:
                    switchRoverOpMode(OpModeEnum::TELEOP);
                break;

                case OpModeEnum::TELEOP:
                    switchRoverOpMode(OpModeEnum::AUTONOMOUS);
                break;

                case OpModeEnum::AUTONOMOUS:
                    switchRoverOpMode(OpModeEnum::STANDBY);
                break;

                default:
                    switchRoverOpMode(OpModeEnum::STANDBY);
                    RCLCPP_ERROR(this->get_logger(), "Operation mode invalid, setting to STANDBY");
            }
            start_debounce_timer_ = this->get_clock()->now();
        }
        
    }

    // Back-button rising-edge cycles through the task modes of the rover
    if(joy_msg->buttons[int(JoyButtons::BACK)] && !joy_last_state_.buttons[int(JoyButtons::BACK)]){
        // Check debounce time
        if((this->get_clock()->now() - back_debounce_timer_).seconds() > 0.1){
            switch(current_rover_task_mode_){
                case TaskModeEnum::IDLE:
                    switchRoverTaskMode(TaskModeEnum::NAV);
                break;

                case TaskModeEnum::NAV:
                    switchRoverTaskMode(TaskModeEnum::EXC);
                break;

                case TaskModeEnum::EXC:
                    switchRoverTaskMode(TaskModeEnum::DMP);
                break;

                case TaskModeEnum::DMP:
                    switchRoverTaskMode(TaskModeEnum::IDLE);
                break;

                default:
                    switchRoverTaskMode(TaskModeEnum::IDLE);
                    RCLCPP_ERROR(this->get_logger(), "Task mode invalid, setting to IDLE");
            }
            back_debounce_timer_ = this->get_clock()->now();
        }
    }

    // A-button rising-edge calls service to start mapping
    if(joy_msg->buttons[int(JoyButtons::A)] && !joy_last_state_.buttons[int(JoyButtons::A)]){
        // Check debounce time
        if((this->get_clock()->now() - a_debounce_timer_).seconds() > 1.0 && current_rover_op_mode_ == OpModeEnum::TELEOP){
            callStartMappingSwitch(true);
        }
        a_debounce_timer_ = this->get_clock()->now();
    }

    // B-button rising-edge calls service to stop mapping
    if(joy_msg->buttons[int(JoyButtons::B)] && !joy_last_state_.buttons[int(JoyButtons::B)]){
        // Check debounce time
        if((this->get_clock()->now() - b_debounce_timer_).seconds() > 1.0 && current_rover_op_mode_ == OpModeEnum::TELEOP){
            callStartMappingSwitch(false);
        }
        b_debounce_timer_ = this->get_clock()->now();
    }

    // Y-button rising-edge calls action to calibrate IMU
    if(joy_msg->buttons[int(JoyButtons::Y)] && !joy_last_state_.buttons[int(JoyButtons::Y)]){
        // Check debounce time
        if((this->get_clock()->now() - y_debounce_timer_).seconds() > 1.0){
            // Check if rover in autonomous mode
            if(current_rover_op_mode_ == OpModeEnum::AUTONOMOUS){
                // Call action to calibrate IMU
                callLocalizationCalibration();
            }
            else{
                RCLCPP_WARN(this->get_logger(), "Rover not in autonomous mode, cannot calibrate IMU");
            }
        }
    }

    // Pass through rover teleop commands
    passRoverTeleopCmd(joy_msg);

    // Store last received joystick state
    setLastJoyState(joy_msg);

}

void ExternalInterface::switchRoverLockStatus(bool mob_val, bool act_val){
    while(!set_params_client_->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_WARN(this->get_logger(), "Waiting for params server to be up...");
    }

    auto set_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    auto mob_param_req = rcl_interfaces::msg::Parameter();
    mob_param_req.name = "rover.mobility_lock";
    mob_param_req.value.type = 1;
    mob_param_req.value.bool_value = mob_val;
    auto act_param_req = rcl_interfaces::msg::Parameter();
    act_param_req.name = "rover.actuation_lock";
    act_param_req.value.type = 1;
    act_param_req.value.bool_value = act_val;

    set_request->parameters = {mob_param_req,act_param_req};

    auto future_result = set_params_client_->async_send_request(set_request);
}

void ExternalInterface::switchRoverOpMode(OpModeEnum mode_to_set){
    while(!set_params_client_->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_WARN(this->get_logger(), "Waiting for params server to be up...");
    }

    auto set_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    auto op_mode_param_req = rcl_interfaces::msg::Parameter();
    op_mode_param_req.name = "rover.op_mode";
    op_mode_param_req.value.type = 2;
    op_mode_param_req.value.integer_value = uint16_t(mode_to_set);

    set_request->parameters = {op_mode_param_req};

    auto future_result = set_params_client_->async_send_request(set_request);
}

void ExternalInterface::switchRoverTaskMode(TaskModeEnum mode_to_set){
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

void ExternalInterface::setLastJoyState(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
    joy_last_state_ = *joy_msg;
}

void ExternalInterface::activeLock(){
    RCLCPP_ERROR(this->get_logger(), "No communication with joystick/control station");
    if(!rover_soft_lock_.mobility_lock || !rover_soft_lock_.actuation_lock){
        // Lock rover if no /joy message received for 3 seconds
        switchRoverLockStatus(true, true);
        // Set to standby
        switchRoverOpMode(OpModeEnum::STANDBY);
    }
}

void ExternalInterface::passRoverTeleopCmd(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
    // Make rover teleop message
    auto rover_teleop_msg = lx_msgs::msg::RoverCommand();
    rover_teleop_msg.mobility_twist.linear.x = joy_msg->axes[int(JoyAxes::LEFT_STICK_V)] * mob_lin_vel_;
    
    // Angular vel sign depends on forward or reverse (inverted)
    if(rover_teleop_msg.mobility_twist.linear.x >= 0.05){
        rover_teleop_msg.mobility_twist.angular.z = joy_msg->axes[int(JoyAxes::LEFT_STICK_H)] * mob_ang_vel_;
    }
    else if(rover_teleop_msg.mobility_twist.linear.x <= -0.05){
        rover_teleop_msg.mobility_twist.angular.z = -joy_msg->axes[int(JoyAxes::LEFT_STICK_H)] * mob_ang_vel_;
    }
    
    // Inverted actuator movement
    rover_teleop_msg.actuator_speed = -joy_msg->axes[int(JoyAxes::RIGHT_STICK_V)];

    // Scale trigger input to [0 to 1] for drum command
    float right_remapped = 0, left_remapped = 0;

    if(joy_msg->axes[int(JoyAxes::RIGHT_TRIG)] <= 0.8){
        right_remapped = remapTrig(joy_msg->axes[int(JoyAxes::RIGHT_TRIG)]);
    }
    if(joy_msg->axes[int(JoyAxes::LEFT_TRIG)] <= 0.8){
        left_remapped = remapTrig(joy_msg->axes[int(JoyAxes::LEFT_TRIG)]);
    }

    rover_teleop_msg.drum_speed = left_remapped - right_remapped;

    // Publish rover teleop
    rover_teleop_publisher_->publish(rover_teleop_msg);
}

double ExternalInterface::remapTrig(float trig_val){
    // Remap trigger value [0.8 to -1] to [0 to 1] for drum command
    float original_range_start = 0;
    float original_range_end = -1.8;
    float remapped_range_start = 0;
    float remapped_range_end = 1;

    trig_val = trig_val - 0.8;

    return (trig_val - original_range_start) / (original_range_end - original_range_start) * (remapped_range_end - remapped_range_start) + remapped_range_start;
}

void ExternalInterface::callStartMappingSwitch(bool map_switch){
    // Call start mapping switch service
    while(!map_switch_client_->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_WARN(this->get_logger(), "Waiting for params server to be up...");
    }

    auto set_request = std::make_shared<lx_msgs::srv::Switch::Request>();

    set_request->switch_state = map_switch;

    RCLCPP_INFO(this->get_logger(), "Calling start mapping service");

    auto future_result = map_switch_client_->async_send_request(set_request, std::bind(&ExternalInterface::mapSwitchCB, this, std::placeholders::_1));
}

void ExternalInterface::mapSwitchCB(rclcpp::Client<lx_msgs::srv::Switch>::SharedFuture future){
    auto status = future.wait_for(std::chrono::milliseconds(100));
    if(status == std::future_status::ready){ 
        if(future.get()->success){
            RCLCPP_INFO(this->get_logger(), "Map status switched");
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Map switch server returned 'unsuccesful'");
        }
    } 
    else{
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}

void ExternalInterface::callLocalizationCalibration(){
    // Call localization calibration action
    while(!calibrate_imu_action_client_->wait_for_action_server(std::chrono::seconds(1))){
        RCLCPP_WARN(this->get_logger(), "Waiting for action server to be up...");
    }

    auto goal_msg = CalibrateImu::Goal();

    RCLCPP_INFO(this->get_logger(), "Calling localization calibration action");

    auto future_goal_handle = calibrate_imu_action_client_->async_send_goal(goal_msg);
}

void ExternalInterface::diagnosticPublish(){
    // Publish diagnostic message
    auto msg = lx_msgs::msg::NodeDiagnostics();
    msg.node_name = this->get_name();
    msg.stamp = this->get_clock()->now();
    diagnostic_publisher_->publish(msg);
}