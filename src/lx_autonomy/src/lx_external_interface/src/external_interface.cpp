#include "lx_external_interface/external_interface.hpp"

ExternalInterface::ExternalInterface(): Node("external_interface_node"){
    // Lock movement at system start
    rover_soft_lock_.mobility_lock = true;
    rover_soft_lock_.actuation_lock = true;

    // Timer for active rover lock
    rover_lock_timer_ = this->create_wall_timer(std::chrono::seconds(3), 
                        std::bind(&ExternalInterface::activeLock, this));

    // Timers for debouncing
    guide_debounce_timer_ = this->get_clock()->now();
    start_debounce_timer_ = this->get_clock()->now();
    back_debounce_timer_ = this->get_clock()->now();
    
    // Set up subscriptions & publishers
    setupCommunications();

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
    rover_teleop_publisher_ = this->create_publisher<lx_msgs::msg::RoverTeleop>("rover_teleop_cmd", 10);

    // Clients
    set_params_client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/param_server_node/set_parameters");
}

void ExternalInterface::setupParams(){
    // Subscriber for global parameter events
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // Callback Lambdas
    auto mob_params_callback = [this](const rclcpp::Parameter & p) {
        RCLCPP_INFO(this->get_logger(), "Parameter updated \"%s\": %s", p.get_name().c_str(), (p.as_bool()?"Locked":"Unlocked"));
        rover_soft_lock_.mobility_lock = p.as_bool();
    };
    auto act_params_callback = [this](const rclcpp::Parameter & p) {
        RCLCPP_INFO(this->get_logger(), "Parameter updated \"%s\": %s", p.get_name().c_str(), (p.as_bool()?"Locked":"Unlocked"));
        rover_soft_lock_.actuation_lock = p.as_bool();
    };
    auto op_mode_params_callback = [this](const rclcpp::Parameter & p) {
        switch(p.as_int()){
            case 0:
               current_rover_op_mode_ = OpModeEnum::STANDBY;
               RCLCPP_INFO(this->get_logger(), "Parameter updated \"%s\": Standby", p.get_name().c_str());
               break;
            case 1:
               current_rover_op_mode_ = OpModeEnum::TELEOP;
               RCLCPP_INFO(this->get_logger(), "Parameter updated \"%s\": Teleop", p.get_name().c_str());
               break;
            case 2:
               current_rover_op_mode_ = OpModeEnum::AUTONOMOUS;
               RCLCPP_INFO(this->get_logger(), "Parameter updated \"%s\": Autonomous", p.get_name().c_str());
               break;
        }
    };
    auto task_mode_params_callback = [this](const rclcpp::Parameter & p){
        switch(p.as_int()){
            case 0:
               current_rover_task_mode_ = TaskModeEnum::IDLE;
               RCLCPP_INFO(this->get_logger(), "Parameter updated \"%s\": Idle", p.get_name().c_str());
               break;
            case 1:
               current_rover_task_mode_ = TaskModeEnum::NAV;
               RCLCPP_INFO(this->get_logger(), "Parameter updated \"%s\": Navigation", p.get_name().c_str());
               break;
            case 2:
               current_rover_task_mode_ = TaskModeEnum::EXC;
               RCLCPP_INFO(this->get_logger(), "Parameter updated \"%s\": Excavation", p.get_name().c_str());
               break;
            case 3:
               current_rover_task_mode_ = TaskModeEnum::DMP;
               RCLCPP_INFO(this->get_logger(), "Parameter updated \"%s\": Dumping", p.get_name().c_str());
               break;
        }
    };
    auto lin_mob_vel_params_callback = [this](const rclcpp::Parameter & p){
        RCLCPP_INFO(this->get_logger(), "Parameter updated \"%s\": %.2f", p.get_name().c_str(), p.as_double());
        mob_lin_vel_ = p.as_double();
    };
    auto ang_mob_vel_params_callback = [this](const rclcpp::Parameter & p){
        RCLCPP_INFO(this->get_logger(), "Parameter updated \"%s\": %.2f", p.get_name().c_str(), p.as_double());
        mob_ang_vel_ = p.as_double();
    };

    // Names of node & params for adding callback
    auto param_server_name = std::string("param_server_node");
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
    // TODO
    auto rover_teleop_msg = lx_msgs::msg::RoverTeleop();
    rover_teleop_msg.mobility_twist.linear.x = joy_msg->axes[int(JoyAxes::LEFT_STICK_V)] * mob_lin_vel_;
    rover_teleop_msg.mobility_twist.angular.z = joy_msg->axes[int(JoyAxes::LEFT_STICK_H)] * mob_ang_vel_;
    rover_teleop_msg.actuator_height.data = joy_msg->axes[int(JoyAxes::RIGHT_STICK_V)];
    rover_teleop_msg.drum_speed.data = joy_msg->axes[int(JoyAxes::LEFT_TRIG)];
    rover_teleop_publisher_->publish(rover_teleop_msg);
}