#include "lx_bringup_autonomy/param_server.hpp"

ParamServer::ParamServer(): Node("param_server_node"){
    initParameters();
}

void ParamServer::initParameters(){
    // Parameters
    this->declare_parameter("rover.mobility_lock", true);
    this->declare_parameter("rover.actuation_lock", true);
    this->declare_parameter("rover.op_mode", 0);
    this->declare_parameter("rover.task_mode", 0);
    this->declare_parameter("limits.max_mob_vel", 0.0);
    this->declare_parameter("limits.min_mob_vel", 0.0);
    this->declare_parameter("limits.max_lin_act_ext", 0.0);
    this->declare_parameter("limits.min_lin_act_ext", 0.0);
    this->declare_parameter("limits.max_drum_speed", 0.0);
    this->declare_parameter("limits.min_drum_speed", 0.0);
    this->declare_parameter("operational.exc_lin_act_ext", 0.0);
    this->declare_parameter("operational.dmp_lin_act_ext", 0.0);
    this->declare_parameter("operational.exc_drum_speed", 0.0);
    this->declare_parameter("operational.dmp_drum_speed", 0.0);

    // Parameter subscriber to listen to any changes
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // Callback Lambda
    auto param_call_back = [this](const rclcpp::Parameter & p) {
        RCLCPP_INFO(
            this->get_logger(), "Parameter updated \"%s\"",
            p.get_name().c_str());
        };

    // Callback handles
    call_back_handle_[0] = param_subscriber_->add_parameter_callback("rover.mobility_lock", param_call_back);
    call_back_handle_[1] = param_subscriber_->add_parameter_callback("rover.actuation_lock", param_call_back);
    call_back_handle_[2] = param_subscriber_->add_parameter_callback("rover.op_mode", param_call_back);
    call_back_handle_[3] = param_subscriber_->add_parameter_callback("rover.task_mode", param_call_back);
    call_back_handle_[4] = param_subscriber_->add_parameter_callback("limits.max_mob_vel", param_call_back);
    call_back_handle_[5] = param_subscriber_->add_parameter_callback("limits.min_mob_vel", param_call_back);
    call_back_handle_[6] = param_subscriber_->add_parameter_callback("limits.max_lin_act_ext", param_call_back);
    call_back_handle_[7] = param_subscriber_->add_parameter_callback("limits.min_lin_act_ext", param_call_back);
    call_back_handle_[8] = param_subscriber_->add_parameter_callback("limits.max_drum_speed", param_call_back);
    call_back_handle_[9] = param_subscriber_->add_parameter_callback("limits.min_drum_speed", param_call_back);
    call_back_handle_[10] = param_subscriber_->add_parameter_callback("operational.exc_lin_act_ext", param_call_back);
    call_back_handle_[11] = param_subscriber_->add_parameter_callback("operational.dmp_lin_act_ext", param_call_back);
    call_back_handle_[12] = param_subscriber_->add_parameter_callback("operational.exc_drum_speed", param_call_back);
    call_back_handle_[13] = param_subscriber_->add_parameter_callback("operational.dmp_drum_speed", param_call_back);
}

