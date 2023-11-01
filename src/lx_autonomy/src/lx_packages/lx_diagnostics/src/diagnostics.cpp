/* Author: Dhruv Tyagi
 * Subscribers:
 * Publishers:
 * Services:
 *
 * - Run diagnostics on robot autonomy and hardware
 * 
 * TODO:
 * - Add diagnostics to autodig, autonav, autodump
 * - Add diagnostics to mapping, localization, planning
 * */

#include "lx_diagnostics/diagnostics.hpp"

Diagnostics::Diagnostics(): Node("diagnostics_node"){
    // Set up subscriptions & publishers
    setupCommunications();

    // Get parameters from the global parameter server
    getParams();

    // Set up parameters from the global parameter server
    setupParams();

    // Set up nodes time tracking
    for(long unsigned int i = 0; i < nodes_list_.size(); i++){
        last_diagnostics_time_.push_back(this->get_clock()->now());
    }

    // Timer
    for(long unsigned int i = 0; i < nodes_list_.size(); i++){
        diagnostics_timers_[i] = this->create_wall_timer(std::chrono::seconds(timeout_period_sec_), std::bind(&Diagnostics::setRoverLock, this));
    }

    RCLCPP_INFO(this->get_logger(), "Diagnostics initialized");
}

void Diagnostics::setupCommunications(){
    // Subscribers
    diagnostics_subscriber_ = this->create_subscription<lx_msgs::msg::NodeDiagnostics>("lx_diagnostics", 10, 
                        std::bind(&Diagnostics::diagnosticsCallBack, this, std::placeholders::_1));
    // Clients
    set_params_client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/param_server_node/set_parameters");
    get_params_client_ = this->create_client<rcl_interfaces::srv::GetParameters>("/param_server_node/get_parameters");
}

void Diagnostics::getParams(){
    while(!get_params_client_->wait_for_service(std::chrono::seconds(2))){
      RCLCPP_INFO(this->get_logger(), "Could not contact param server");
      return;
    }

    // Get important parameters
    auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    get_request->names = {"rover.mobility_lock", "rover.actuation_lock", "rover.op_mode"};

    // Send request
    auto param_result_ = get_params_client_->async_send_request(get_request,std::bind(&Diagnostics::paramCB, this, std::placeholders::_1));
}

void Diagnostics::paramCB(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future){
    auto status = future.wait_for(std::chrono::milliseconds(100));
    // If request successful, save all params in global variables
    if (status == std::future_status::ready) {
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
    } 
    else {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}

void Diagnostics::setupParams(){
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

    // Names of node & params for adding callback
    auto param_server_name = std::string("param_server_node");
    auto mob_lock_param_name = std::string("rover.mobility_lock");
    auto act_lock_param_name = std::string("rover.actuation_lock");
    auto op_mode_param_name = std::string("rover.op_mode");

    // Store callback handles for each parameter
    mob_param_cb_handle_ = param_subscriber_->add_parameter_callback(mob_lock_param_name, mob_params_callback, param_server_name);
    act_param_cb_handle_ = param_subscriber_->add_parameter_callback(act_lock_param_name, act_params_callback, param_server_name);
    op_mode_param_cb_handle_ = param_subscriber_->add_parameter_callback(op_mode_param_name, op_mode_params_callback, param_server_name);
}

void Diagnostics::diagnosticsCallBack(const lx_msgs::msg::NodeDiagnostics::SharedPtr msg){
    // Find the index of the node in the nodes_list_
    int node_index = std::find(nodes_list_.begin(), nodes_list_.end(), msg->node_name) - nodes_list_.begin();
    
    // Respective timer reset
    diagnostics_timers_[node_index]->reset();

    // Update time of last diagnostics message
    last_diagnostics_time_[node_index] = this->get_clock()->now();
}

void Diagnostics::setRoverLock(){
    RCLCPP_ERROR(this->get_logger(), "DIAGNOSTICS TIMEOUT, Dead node(s):");

    // Find and display dead node(s)
    for(long unsigned int i = 0; i < nodes_list_.size(); i++){
        if((this->get_clock()->now() - last_diagnostics_time_[i]).seconds() > timeout_period_sec_){
            RCLCPP_ERROR(this->get_logger(), "%s ", nodes_list_[i].c_str());
        }
    }
    
    if((!rover_soft_lock_.mobility_lock || !rover_soft_lock_.actuation_lock) && (current_rover_op_mode_ == OpModeEnum::AUTONOMOUS)){
        RCLCPP_ERROR(this->get_logger(), "Locking Rover");
        // Lock rover if some diagnostics message not received for timeout period
        while(!set_params_client_->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_WARN(this->get_logger(), "Waiting for params server to be up...");
        }

        auto set_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        auto mob_param_req = rcl_interfaces::msg::Parameter();
        mob_param_req.name = "rover.mobility_lock";
        mob_param_req.value.type = 1;
        mob_param_req.value.bool_value = true;
        auto act_param_req = rcl_interfaces::msg::Parameter();
        act_param_req.name = "rover.actuation_lock";
        act_param_req.value.type = 1;
        act_param_req.value.bool_value = true;

        set_request->parameters = {mob_param_req, act_param_req};

        auto future_result = set_params_client_->async_send_request(set_request);
    }
}