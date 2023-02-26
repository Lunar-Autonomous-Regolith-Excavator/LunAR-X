#include "lx_bringup_autonomy/param_server.hpp"

ParamServer::ParamServer(): Node("param_server_node"){
    initParameters();
}

void ParamServer::initParameters(){
    // Parameters
    this->declare_parameter("mobility_lock", true);
    this->declare_parameter("actuation_lock", true);
    this->declare_parameter("op_mode", 0);
    this->declare_parameter("task_mode", 0);

    // Parameter subscriber to listen to any changes
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    auto cb = [this](const rclcpp::Parameter & p) {
        RCLCPP_INFO(
            this->get_logger(), "Received an update to parameter \"%s\"",
            p.get_name().c_str());
        };

    cb_handle_[0] = param_subscriber_->add_parameter_callback("mobility_lock", cb);
    cb_handle_[1] = param_subscriber_->add_parameter_callback("actuation_lock", cb);
    cb_handle_[2] = param_subscriber_->add_parameter_callback("op_mode", cb);
    cb_handle_[3] = param_subscriber_->add_parameter_callback("task_mode", cb);
}

