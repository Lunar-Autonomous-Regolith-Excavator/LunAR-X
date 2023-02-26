#include "lx_bringup_autonomy/param_server.hpp"

ParamServer::ParamServer(): Node("param_server_node"){
    initParameters();
}

void ParamServer::initParameters(){
    this->declare_parameter("example_param", 0.2);

    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    auto cb = [this](const rclcpp::Parameter & p) {
        RCLCPP_INFO(
            this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s: \"%ld\"",
            p.get_name().c_str(),
            p.get_type_name().c_str(),
            p.as_int());
        };

    cb_handle_ = param_subscriber_->add_parameter_callback("an_int_param", cb);
}

