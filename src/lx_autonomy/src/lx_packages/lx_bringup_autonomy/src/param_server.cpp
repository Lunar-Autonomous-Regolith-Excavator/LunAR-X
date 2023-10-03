/* Author: Dhruv Tyagi
 * - Provides access to globally crucial variables like current lock status, operation & task modes, actuation limits etc. 
 * - Refer to lx_bringup_autonomy/config/params.yaml for all global params
 * */

#include "lx_bringup_autonomy/param_server.hpp"

ParamServer::ParamServer(): Node("param_server_node"){
    setupCommunications();

    diagnostic_pub_timer_ = this->create_wall_timer(std::chrono::seconds(diagnostic_pub_period_), std::bind(&ParamServer::diagnosticPublish, this));

    initParameters();
}

void ParamServer::setupCommunications(){
    // Publishers
    diagnostic_publisher_ = this->create_publisher<lx_msgs::msg::NodeDiagnostics>("lx_diagnostics", 10);
}

void ParamServer::initParameters(){
    // Parameters
    this->declare_parameter("rover.mobility_lock", true);
    this->declare_parameter("rover.actuation_lock", true);
    this->declare_parameter("rover.op_mode", 0);
    this->declare_parameter("rover.task_mode", 0);
    this->declare_parameter("limits.max_lin_mob_vel", 0.0);
    this->declare_parameter("limits.max_lin_mob_acc", 0.0);
    this->declare_parameter("limits.max_ang_mob_vel", 0.0);
    this->declare_parameter("limits.max_lin_act_ext", 0.0);
    this->declare_parameter("limits.min_lin_act_ext", 0.0);
    this->declare_parameter("limits.max_drum_speed", 0.0);
    this->declare_parameter("limits.min_drum_speed", 0.0);
    this->declare_parameter("operational.nav_mob_vel", 0.0);
    this->declare_parameter("operational.exc_mob_vel", 0.0);
    this->declare_parameter("operational.dmp_mob_vel", 0.0);
    this->declare_parameter("operational.exc_lin_act_ext", 0.0);
    this->declare_parameter("operational.dmp_lin_act_ext", 0.0);
    this->declare_parameter("operational.exc_drum_speed", 0.0);
    this->declare_parameter("operational.dmp_drum_speed", 0.0);
    this->declare_parameter("autodig.pid_outer", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter("autodig.pid_inner", std::vector<double>{0.0, 0.0, 0.0});

    // Parameter subscriber to listen to any changes
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // Callback Lambdas
    auto param_bool_call_back = [this](const rclcpp::Parameter & p) {
            RCLCPP_INFO(this->get_logger(), "Parameter updated - \"%s\": %s", p.get_name().c_str(), (p.as_bool()?"TRUE":"FALSE"));
        };
    auto param_int_call_back = [this](const rclcpp::Parameter & p) {
            RCLCPP_INFO(this->get_logger(), "Parameter updated - \"%s\": %ld", p.get_name().c_str(), p.as_int());
        };
    auto param_double_call_back = [this](const rclcpp::Parameter & p) {
            RCLCPP_INFO(this->get_logger(), "Parameter updated - \"%s\": %.2f", p.get_name().c_str(), p.as_double());
        };
    auto param_pid_call_back = [this](const rclcpp::Parameter & p) {
        RCLCPP_INFO(this->get_logger(), "Parameter updated - \"%s\": [%.3f, %.5f, %.3f]", p.get_name().c_str(), p.as_double_array()[0], p.as_double_array()[1], p.as_double_array()[2]);
    };

    // Callback handles
    call_back_handle_[0] = param_subscriber_->add_parameter_callback("rover.mobility_lock", param_bool_call_back);
    call_back_handle_[1] = param_subscriber_->add_parameter_callback("rover.actuation_lock", param_bool_call_back);
    call_back_handle_[2] = param_subscriber_->add_parameter_callback("rover.op_mode", param_int_call_back);
    call_back_handle_[3] = param_subscriber_->add_parameter_callback("rover.task_mode", param_int_call_back);
    call_back_handle_[4] = param_subscriber_->add_parameter_callback("limits.max_lin_mob_vel", param_double_call_back);
    call_back_handle_[5] = param_subscriber_->add_parameter_callback("limits.max_lin_mob_acc", param_double_call_back);
    call_back_handle_[6] = param_subscriber_->add_parameter_callback("limits.max_ang_mob_vel", param_double_call_back);
    call_back_handle_[7] = param_subscriber_->add_parameter_callback("limits.max_lin_act_ext", param_double_call_back);
    call_back_handle_[8] = param_subscriber_->add_parameter_callback("limits.min_lin_act_ext", param_double_call_back);
    call_back_handle_[9] = param_subscriber_->add_parameter_callback("limits.max_drum_speed", param_double_call_back);
    call_back_handle_[10] = param_subscriber_->add_parameter_callback("limits.min_drum_speed", param_double_call_back);
    call_back_handle_[11] = param_subscriber_->add_parameter_callback("operational.nav_mob_vel", param_double_call_back);
    call_back_handle_[12] = param_subscriber_->add_parameter_callback("operational.exc_mob_vel", param_double_call_back);
    call_back_handle_[13] = param_subscriber_->add_parameter_callback("operational.dmp_mob_vel", param_double_call_back);
    call_back_handle_[14] = param_subscriber_->add_parameter_callback("operational.exc_lin_act_ext", param_double_call_back);
    call_back_handle_[15] = param_subscriber_->add_parameter_callback("operational.dmp_lin_act_ext", param_double_call_back);
    call_back_handle_[16] = param_subscriber_->add_parameter_callback("operational.exc_drum_speed", param_double_call_back);
    call_back_handle_[17] = param_subscriber_->add_parameter_callback("operational.dmp_drum_speed", param_double_call_back);
    call_back_handle_[18] = param_subscriber_->add_parameter_callback("autodig.pid_outer", param_pid_call_back);
    call_back_handle_[19] = param_subscriber_->add_parameter_callback("autodig.pid_inner", param_pid_call_back);
    
}

void ParamServer::diagnosticPublish(){
    // Publish diagnostic message
    auto msg = lx_msgs::msg::NodeDiagnostics();
    msg.node_name = this->get_name();
    msg.stamp = this->get_clock()->now();
    diagnostic_publisher_->publish(msg);
}
