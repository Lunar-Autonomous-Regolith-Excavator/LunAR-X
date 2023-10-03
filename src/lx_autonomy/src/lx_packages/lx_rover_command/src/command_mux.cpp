/* Author: Dhruv Tyagi
 * Subscribers:
 *    - /rover_teleop_cmd: [lx_msgs::msg::RoverCommand] Teleop command from joystick via external_interface_node
 *    - /rover_auto_cmd: [lx_msgs::msg::RoverCommand] Autonomy command
 * Publishers:
 *    - /rover_hw_cmd: [lx_msgs::msg::RoverCommand] Command published to the lx_hardware container's hardware_mux_node
 *
 * - Switches between teleop and autonomous commands based on the operation mode
 * - Enforces actuation limits
 * 
 * TODO
 * - Test autonomous input handling
 * - Test acceleration clipping
 * */

#include "lx_rover_command/command_mux.hpp"

CommandMux::CommandMux(): Node("command_mux_node"){
    // Lock movement at system start
    rover_soft_lock_.mobility_lock = true;
    rover_soft_lock_.actuation_lock = true;
    
    // Set up subscriptions & publishers
    setupCommunications();

    // Timer for diagnostics publisher
    diagnostic_pub_timer_ = this->create_wall_timer(std::chrono::seconds(diagnostic_pub_period_), 
                        std::bind(&CommandMux::diagnosticPublish, this));

    // Get parameters from the global parameter server
    getParams();

    // Set up parameters from the global parameter server
    setupParams();

    RCLCPP_INFO(this->get_logger(), "Command Mux initialized");
} 

void CommandMux::setupCommunications(){
    // Subscribers
    rover_teleop_subscriber_ = this->create_subscription<lx_msgs::msg::RoverCommand>("rover_teleop_cmd", 10 , 
                            std::bind(&CommandMux::roverTeleopCallBack, this, std::placeholders::_1));
    rover_auto_subscriber_ = this->create_subscription<lx_msgs::msg::RoverCommand>("rover_auto_cmd", 10 , 
                            std::bind(&CommandMux::roverAutoCallBack, this, std::placeholders::_1));

    // Publishers
    rover_hw_cmd_publisher_ = this->create_publisher<lx_msgs::msg::RoverCommand>("rover_hw_cmd", 10);
    diagnostic_publisher_ = this->create_publisher<lx_msgs::msg::NodeDiagnostics>("diagnostics", 10);

    // Clients
    get_params_client_ = this->create_client<rcl_interfaces::srv::GetParameters>("/param_server_node/get_parameters");
}

void CommandMux::getParams(){
    while(!get_params_client_->wait_for_service(std::chrono::seconds(2))){
      RCLCPP_INFO(this->get_logger(), "Could not contact param server");
      return;
    }

    // Get important parameters
    auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    get_request->names = {"rover.mobility_lock", "rover.actuation_lock", 
                          "rover.op_mode", "rover.task_mode", 
                          "limits.max_lin_mob_vel", "limits.max_ang_mob_vel",
                          "limits.max_drum_speed", "limits.max_lin_mob_acc"};

    // Send request
    auto param_result_ = get_params_client_->async_send_request(get_request,std::bind(&CommandMux::paramCB, this, std::placeholders::_1));
}

void CommandMux::paramCB(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future){
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

        max_mob_lin_vel_ = future.get()->values.at(4).double_value;
        RCLCPP_DEBUG(this->get_logger(), "Parameter set Max Linear Vel: %.2f", max_mob_lin_vel_);
        max_mob_ang_vel_ = future.get()->values.at(5).double_value;
        RCLCPP_DEBUG(this->get_logger(), "Parameter set Max Angular Vel: %.2f", max_mob_ang_vel_);
        max_drum_speed_ = future.get()->values.at(6).double_value;
        RCLCPP_DEBUG(this->get_logger(), "Parameter set Max Drum Speed: %.2f", max_drum_speed_);
        max_mob_lin_acc_ = future.get()->values.at(7).double_value;
        RCLCPP_DEBUG(this->get_logger(), "Parameter set Max Mob Linear Acceleration: %.2f", max_mob_lin_acc_);
    } 
    else {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
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
    auto lin_mob_acc_params_callback = [this](const rclcpp::Parameter & p){
        RCLCPP_DEBUG(this->get_logger(), "Parameter updated \"%s\": %.2f", p.get_name().c_str(), p.as_double());
        max_mob_lin_acc_ = p.as_double();
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
    auto lin_mob_acc_param_name = std::string("limits.max_lin_mob_acc");
    auto ang_mob_vel_param_name = std::string("limits.max_ang_mob_vel");
    auto drum_speed_param_name = std::string("limits.max_drum_speed");

    // Store callback handles for each parameter
    mob_param_cb_handle_ = param_subscriber_->add_parameter_callback(mob_lock_param_name, mob_params_callback, param_server_name);
    act_param_cb_handle_ = param_subscriber_->add_parameter_callback(act_lock_param_name, act_params_callback, param_server_name);
    op_mode_param_cb_handle_ = param_subscriber_->add_parameter_callback(op_mode_param_name, op_mode_params_callback, param_server_name);
    task_mode_param_cb_handle_ = param_subscriber_->add_parameter_callback(task_mode_param_name, task_mode_params_callback, param_server_name);
    lin_mob_vel_param_cb_handle_ = param_subscriber_->add_parameter_callback(lin_mob_vel_param_name, lin_mob_vel_params_callback, param_server_name);
    lin_mob_acc_param_cb_handle_ = param_subscriber_->add_parameter_callback(lin_mob_acc_param_name, lin_mob_acc_params_callback, param_server_name);
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

void CommandMux::roverAutoCallBack(const lx_msgs::msg::RoverCommand::SharedPtr rover_auto_msg){
    // Passthrough auto command 
    auto_cmd_pub_thread_ = std::thread(std::bind(&CommandMux::autoPassthrough, this, rover_auto_msg));

    // Have to detach thread before it goes out of scope
    auto_cmd_pub_thread_.detach(); 

}

void CommandMux::autoPassthrough(const lx_msgs::msg::RoverCommand::SharedPtr rover_auto_msg){
    // If current op_mode is autonomous & task_mode is not idle
    if(current_rover_op_mode_ == OpModeEnum::AUTONOMOUS /*&& current_rover_task_mode_ != TaskModeEnum::IDLE*/){
        // Pass through autonomy command
        sendCmdToHardware(rover_auto_msg);
    } 
}

void CommandMux::sendCmdToHardware(const lx_msgs::msg::RoverCommand::SharedPtr received_msg){

    auto cmd_msg = lx_msgs::msg::RoverCommand();

    // If rover is not locked, pass the commands
    if(!rover_soft_lock_.mobility_lock){
        // Clip mobility linear command to [-max_mob_lin_vel_  max_mob_lin_vel_]
        if(received_msg->mobility_twist.linear.x > 0.0){
            cmd_msg.mobility_twist.linear.x = (received_msg->mobility_twist.linear.x > max_mob_lin_vel_ ? max_mob_lin_vel_ : received_msg->mobility_twist.linear.x);
        }
        else{
            cmd_msg.mobility_twist.linear.x = (received_msg->mobility_twist.linear.x < -max_mob_lin_vel_ ? -max_mob_lin_vel_ : received_msg->mobility_twist.linear.x);
        }
        // Add acceleration clipping
        if(abs(cmd_msg.mobility_twist.linear.x - last_mob_lin_vel_) > max_mob_lin_acc_){
            if(cmd_msg.mobility_twist.linear.x > last_mob_lin_vel_){
                cmd_msg.mobility_twist.linear.x = last_mob_lin_vel_ + max_mob_lin_acc_;
            }
            else{
                cmd_msg.mobility_twist.linear.x = last_mob_lin_vel_ - max_mob_lin_acc_;
            }
        }
        last_mob_lin_vel_ = cmd_msg.mobility_twist.linear.x;
        
        // Clip mobility angular command to [-max_mob_ang_vel_  max_mob_ang_vel_]
        if(received_msg->mobility_twist.angular.z > 0.0){
            cmd_msg.mobility_twist.angular.z = (received_msg->mobility_twist.angular.z > max_mob_ang_vel_ ? max_mob_ang_vel_ : received_msg->mobility_twist.angular.z);
        }
        else{
            cmd_msg.mobility_twist.angular.z = (received_msg->mobility_twist.angular.z < -max_mob_ang_vel_ ? -max_mob_ang_vel_ : received_msg->mobility_twist.angular.z);
        }        
    }
    if(!rover_soft_lock_.actuation_lock){
        // Clip actuator command to [-1 1]
        if(received_msg->actuator_speed > 0.0){
            cmd_msg.actuator_speed = (received_msg->actuator_speed > 1.0 ? 1.0 : received_msg->actuator_speed);
        }
        else{
            cmd_msg.actuator_speed = (received_msg->actuator_speed < -1.0 ? -1.0 : received_msg->actuator_speed);
        }

        // Clip drum command to [-1 1]
        if(received_msg->drum_speed > 0.0){
            cmd_msg.drum_speed = (received_msg->drum_speed > 1.0 ? 1.0 : received_msg->drum_speed);
        }
        else{
            cmd_msg.drum_speed = (received_msg->drum_speed < -1.0 ? -1.0 : received_msg->drum_speed);
        }
    }

    rover_hw_cmd_publisher_->publish(cmd_msg);
}

void CommandMux::sendCmdToHardware(geometry_msgs::msg::Twist& twist_msg, float& linact_msg, float& drum_msg){

    auto cmd_msg = lx_msgs::msg::RoverCommand();

    // If rover is not locked, pass the commands
    if(!rover_soft_lock_.mobility_lock){
        // Clip mobility linear command to [-max_mob_lin_vel_  max_mob_lin_vel_]
        if(twist_msg.linear.x > 0.0){
            cmd_msg.mobility_twist.linear.x = (twist_msg.linear.x > max_mob_lin_vel_ ? max_mob_lin_vel_ : twist_msg.linear.x);
        }
        else{
            cmd_msg.mobility_twist.linear.x = (twist_msg.linear.x < -max_mob_lin_vel_ ? -max_mob_lin_vel_ : twist_msg.linear.x);
        }
        // Add acceleration clipping
        if(abs(cmd_msg.mobility_twist.linear.x - last_mob_lin_vel_) > max_mob_lin_acc_){
            if(cmd_msg.mobility_twist.linear.x > last_mob_lin_vel_){
                cmd_msg.mobility_twist.linear.x = last_mob_lin_vel_ + max_mob_lin_acc_;
            }
            else{
                cmd_msg.mobility_twist.linear.x = last_mob_lin_vel_ - max_mob_lin_acc_;
            }
        }
        last_mob_lin_vel_ = cmd_msg.mobility_twist.linear.x;
        
        // Clip mobility angular command to [-max_mob_ang_vel_  max_mob_ang_vel_]
        if(twist_msg.angular.z > 0.0){
            cmd_msg.mobility_twist.angular.z = (twist_msg.angular.z > max_mob_ang_vel_ ? max_mob_ang_vel_ : twist_msg.angular.z);
        }
        else{
            cmd_msg.mobility_twist.angular.z = (twist_msg.angular.z < -max_mob_ang_vel_ ? -max_mob_ang_vel_ : twist_msg.angular.z);
        }        
    }
    if(!rover_soft_lock_.actuation_lock){
        // Clip actuator command to [-1 1]
        if(linact_msg > 0.0){
            cmd_msg.actuator_speed = (linact_msg > 1.0 ? 1.0 : linact_msg);
        }
        else{
            cmd_msg.actuator_speed = (linact_msg < -1.0 ? -1.0 : linact_msg);
        }

        // Clip drum command to [-max_drum_speed_  max_drum_speed_]
        if(drum_msg > 0.0){
            cmd_msg.drum_speed = (drum_msg > 1.0 ? 1.0 : drum_msg);
        }
        else{
            cmd_msg.drum_speed = (drum_msg < -1.0 ? -1.0 : drum_msg);
        }
    }

    rover_hw_cmd_publisher_->publish(cmd_msg);
}

void CommandMux::diagnosticPublish(){
    // Publish diagnostic message
    auto msg = lx_msgs::msg::NodeDiagnostics();
    msg.node_name = this->get_name();
    msg.stamp = this->get_clock()->now();
    diagnostic_publisher_->publish(msg);
}