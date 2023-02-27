#include "lx_rover_command/command_mux.hpp"

CommandMux::CommandMux(): Node("command_mux_node"){
    // Lock movement at system start
    rover_soft_lock_.mobility_lock = true;
    rover_soft_lock_.actuation_lock = true;
    
    // Set up subscriptions & publishers
    setupCommunications();

    // Set up parameters from the global parameter server
    setupParams();

    RCLCPP_INFO(this->get_logger(), "Command Mux initialized");
} 

void CommandMux::setupCommunications(){
    // Subscribers
    rover_teleop_subscriber_ = this->create_subscription<lx_msgs::msg::RoverCommand>("rover_teleop_cmd", 10 , 
                            std::bind(&CommandMux::roverTeleopCallBack, this, std::placeholders::_1));

    // Publishers
    rover_hw_cmd_publisher_ = this->create_publisher<lx_msgs::msg::RoverCommand>("rover_hw_cmd", 10);
}

void CommandMux::setupParams(){
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
        max_mob_lin_vel_ = p.as_double();
    };
    auto ang_mob_vel_params_callback = [this](const rclcpp::Parameter & p){
        RCLCPP_DEBUG(this->get_logger(), "Parameter updated \"%s\": %.2f", p.get_name().c_str(), p.as_double());
        max_mob_ang_vel_ = p.as_double();
    };
    auto drum_speed_params_callback = [this](const rclcpp::Parameter & p){
        RCLCPP_DEBUG(this->get_logger(), "Parameter updated \"%s\": %.2f", p.get_name().c_str(), p.as_double());
        max_drum_speed_ = p.as_double();
    };

    // Names of node & params for adding callback
    auto param_server_name = std::string("param_server_node");
    auto mob_lock_param_name = std::string("rover.mobility_lock");
    auto act_lock_param_name = std::string("rover.actuation_lock");
    auto op_mode_param_name = std::string("rover.op_mode");
    auto task_mode_param_name = std::string("rover.task_mode");
    auto lin_mob_vel_param_name = std::string("limits.max_lin_mob_vel");
    auto ang_mob_vel_param_name = std::string("limits.max_ang_mob_vel");
    auto drum_speed_param_name = std::string("limits.max_drum_speed");

    // Store callback handles for each parameter
    mob_param_cb_handle_ = param_subscriber_->add_parameter_callback(mob_lock_param_name, mob_params_callback, param_server_name);
    act_param_cb_handle_ = param_subscriber_->add_parameter_callback(act_lock_param_name, act_params_callback, param_server_name);
    op_mode_param_cb_handle_ = param_subscriber_->add_parameter_callback(op_mode_param_name, op_mode_params_callback, param_server_name);
    task_mode_param_cb_handle_ = param_subscriber_->add_parameter_callback(task_mode_param_name, task_mode_params_callback, param_server_name);
    lin_mob_vel_param_cb_handle_ = param_subscriber_->add_parameter_callback(lin_mob_vel_param_name, lin_mob_vel_params_callback, param_server_name);
    ang_mob_vel_param_cb_handle_ = param_subscriber_->add_parameter_callback(ang_mob_vel_param_name, ang_mob_vel_params_callback, param_server_name);
    drum_speed_param_cb_handle_ = param_subscriber_->add_parameter_callback(drum_speed_param_name, drum_speed_params_callback, param_server_name);
}

void CommandMux::roverTeleopCallBack(const lx_msgs::msg::RoverCommand::SharedPtr rover_teleop_msg){

    // Passthrough teleop command 
    teleop_cmd_pub_thread_ = std::thread(std::bind(&CommandMux::teleopPassthrough, this, rover_teleop_msg));

    // Have to detach thread before it goes out of scope
    teleop_cmd_pub_thread_.detach(); 

}

void CommandMux::teleopPassthrough(const lx_msgs::msg::RoverCommand::SharedPtr rover_teleop_msg){
    // If current op_mode is teleop & task_mode is not idle
    if(current_rover_op_mode_ == OpModeEnum::TELEOP && current_rover_task_mode_ != TaskModeEnum::IDLE){
        // Pass through teleop command
        sendCmdToHardware(rover_teleop_msg);
    } 
}

void CommandMux::sendCmdToHardware(const lx_msgs::msg::RoverCommand::SharedPtr rover_teleop_msg){

    auto cmd_msg = lx_msgs::msg::RoverCommand();

    // If rover is not locked, pass the commands
    if(!rover_soft_lock_.mobility_lock){
        cmd_msg.mobility_twist.linear.x = (abs(rover_teleop_msg->mobility_twist.linear.x) > max_mob_lin_vel_ ? max_mob_lin_vel_ : rover_teleop_msg->mobility_twist.linear.x);
        cmd_msg.mobility_twist.angular.z = (abs(rover_teleop_msg->mobility_twist.angular.z) > max_mob_ang_vel_ ? max_mob_ang_vel_ : rover_teleop_msg->mobility_twist.angular.z);
    }
    if(!rover_soft_lock_.actuation_lock){
        cmd_msg.actuator_height.data = (abs(rover_teleop_msg->actuator_height.data) > 1.0 ? 1.0 : rover_teleop_msg->actuator_height.data);
        cmd_msg.drum_speed.data = (abs(rover_teleop_msg->drum_speed.data) > max_drum_speed_ ? max_drum_speed_ : rover_teleop_msg->drum_speed.data);
    }

    rover_hw_cmd_publisher_->publish(cmd_msg);
}

void CommandMux::sendCmdToHardware(geometry_msgs::msg::Twist& twist_msg, lx_msgs::msg::LinActHeight& linact_msg, lx_msgs::msg::DrumSpeed& drum_msg){

    auto cmd_msg = lx_msgs::msg::RoverCommand();

    // If rover is not locked, pass the commands
    if(!rover_soft_lock_.mobility_lock){
        cmd_msg.mobility_twist.linear.x = (abs(twist_msg.linear.x) > max_mob_lin_vel_ ? max_mob_lin_vel_ : twist_msg.linear.x);
        cmd_msg.mobility_twist.angular.z = (abs(twist_msg.angular.z) > max_mob_ang_vel_ ? max_mob_ang_vel_ : twist_msg.angular.z);
    }
    if(!rover_soft_lock_.actuation_lock){
        cmd_msg.actuator_height.data = (abs(linact_msg.data) > 1.0 ? 1.0 : linact_msg.data);
        cmd_msg.drum_speed.data = (abs(drum_msg.data) > max_drum_speed_ ? max_drum_speed_ : drum_msg.data);
    }

    rover_hw_cmd_publisher_->publish(cmd_msg);
}