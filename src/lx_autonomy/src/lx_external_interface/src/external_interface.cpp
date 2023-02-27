#include "lx_external_interface/external_interface.hpp"

ExternalInterface::ExternalInterface(): Node("external_interface_node"){
    // Lock movement at system start
    rover_soft_lock_.mobility_lock = true;
    rover_soft_lock_.actuation_lock = true;

    // Set rover to standby at system start
    current_rover_op_mode_ = OpModeEnum::STANDBY;

    // Timer for active rover lock
    rover_lock_timer_ = this->create_wall_timer(std::chrono::seconds(3), 
                        std::bind(&ExternalInterface::activeLock, this));
    guide_debounce_timer_ = this->get_clock()->now();
    
    // Set up subscriptions & publishers
    setupCommunications();

    // Set up parameters from the global parameter server
    setupParams();

    // Publish rover lock
    switchLockStatus(true, true);

    RCLCPP_INFO(this->get_logger(), "External Interface initialized");
}

void ExternalInterface::setupCommunications(){
    // Subscribers
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10 , 
                            std::bind(&ExternalInterface::joyCallBack, this, std::placeholders::_1));

    // Publishers
    // rover_mode_publisher_ = this->create_publisher<lx_msgs::msg::RoverOpMode>("rover_op_mode", 1);
    rover_teleop_publisher_ = this->create_publisher<lx_msgs::msg::RoverTeleop>("rover_teleop_cmd", 10);

    // Clients
    set_params_client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/param_server_node/set_parameters");
}

void ExternalInterface::setupParams(){
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

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
    auto param_server_name = std::string("param_server_node");
    auto mob_lock_param_name = std::string("rover.mobility_lock");
    auto act_lock_param_name = std::string("rover.actuation_lock");
    auto op_mode_param_name = std::string("rover.op_mode");
    auto task_mode_param_name = std::string("rover.task_mode");
    mob_param_cb_handle_ = param_subscriber_->add_parameter_callback(mob_lock_param_name, mob_params_callback, param_server_name);
    act_param_cb_handle_ = param_subscriber_->add_parameter_callback(act_lock_param_name, act_params_callback, param_server_name);
    op_mode_param_cb_handle_ = param_subscriber_->add_parameter_callback(op_mode_param_name, op_mode_params_callback, param_server_name);
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
            switchLockStatus(!rover_soft_lock_.mobility_lock,!rover_soft_lock_.actuation_lock);
            guide_debounce_timer_ = this->get_clock()->now();
        }
    }

    // Start-button rising-edge cycles through the operating modes of the rover
    if(joy_msg->buttons[int(JoyButtons::START)] && !joy_last_state_.buttons[int(JoyButtons::START)]){
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
                RCLCPP_ERROR(this->get_logger(), "Rover mode invalid, setting to STANDBY");
        }
    }

    // Pass through rover teleop commands
    passRoverTeleopCmd(joy_msg);

    // Store last received joystick state
    setLastJoyState(joy_msg);

}

void ExternalInterface::switchLockStatus(bool mob_val, bool act_val){
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

void ExternalInterface::setLastJoyState(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
    joy_last_state_ = *joy_msg;
}

void ExternalInterface::activeLock(){
    RCLCPP_ERROR(this->get_logger(), "No communication with joystick/control station");
    if(!rover_soft_lock_.mobility_lock || !rover_soft_lock_.actuation_lock){
        // Lock rover if no /joy message received for 3 seconds
        switchLockStatus(true, true);
        // Set to standby
        switchRoverOpMode(OpModeEnum::STANDBY);
    }
}

void ExternalInterface::passRoverTeleopCmd(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
    // TODO
}