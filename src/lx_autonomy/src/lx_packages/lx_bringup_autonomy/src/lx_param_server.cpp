/* Author: Dhruv Tyagi
 * - Provides access to globally crucial variables like current lock status, operation & task modes, actuation limits etc. 
 * - Refer to lx_bringup_autonomy/config/params.yaml for all global params
 * */

#include "lx_bringup_autonomy/lx_param_server.hpp"

ParamServer::ParamServer(): Node("lx_param_server_node"){
    setupCommunications();

    diagnostic_pub_timer_ = this->create_wall_timer(std::chrono::seconds(diagnostic_pub_period_), std::bind(&ParamServer::diagnosticPublish, this));

    initParameters();
}

void ParamServer::setupCommunications(){
    // Publishers
    diagnostic_publisher_ = this->create_publisher<lx_msgs::msg::NodeDiagnostics>("lx_diagnostics", 10);
    modes_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("operations/rover_modes", 10);
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

    // Parameter subscriber to listen to any changes
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // Callback Lambdas
    auto param_bool_call_back = [this](const rclcpp::Parameter & p) {
            RCLCPP_INFO(this->get_logger(), "Parameter updated - \"%s\": %s", p.get_name().c_str(), (p.as_bool()?"TRUE":"FALSE"));
            if(p.get_name() == "rover.mobility_lock")
                rover_lock_mobility_ = p.as_bool();
            else if(p.get_name() == "rover.actuation_lock")
                rover_lock_actuation_ = p.as_bool();
            
            updateModeMarker();
        };
    auto param_int_call_back = [this](const rclcpp::Parameter & p) {
            RCLCPP_INFO(this->get_logger(), "Parameter updated - \"%s\": %ld", p.get_name().c_str(), p.as_int());
            if(p.get_name() == "rover.op_mode")
                rover_op_mode_ = p.as_int();
            else if(p.get_name() == "rover.task_mode")
                rover_task_mode_ = p.as_int();
            
            updateModeMarker();
        };
    auto param_double_call_back = [this](const rclcpp::Parameter & p) {
            RCLCPP_INFO(this->get_logger(), "Parameter updated - \"%s\": %.2f", p.get_name().c_str(), p.as_double());
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
}

void ParamServer::diagnosticPublish(){
    // Publish diagnostic message
    auto msg = lx_msgs::msg::NodeDiagnostics();
    msg.node_name = this->get_name();
    msg.stamp = this->get_clock()->now();
    diagnostic_publisher_->publish(msg);
}

void ParamServer::updateModeMarker(){
    // Publish rover modes marker
    auto msg = visualization_msgs::msg::MarkerArray();
    visualization_msgs::msg::Marker lock_marker;
    lock_marker.header.frame_id = "map";
    lock_marker.header.stamp = this->get_clock()->now();
    lock_marker.ns = "lock_mode";
    lock_marker.id = 0;
    lock_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    lock_marker.action = visualization_msgs::msg::Marker::ADD;
    lock_marker.pose.position.x = 6.3;
    lock_marker.pose.position.y = 6.8;
    lock_marker.pose.position.z = 0.0;
    lock_marker.pose.orientation.x = 0.0;
    lock_marker.pose.orientation.y = 0.0;
    lock_marker.pose.orientation.z = 0.0;
    lock_marker.pose.orientation.w = 1.0;
    lock_marker.scale.x = 0.2;
    lock_marker.scale.y = 0.2;
    lock_marker.scale.z = 0.2;
    if(rover_lock_mobility_ && rover_lock_actuation_){
        // Color red
        lock_marker.color.r = 1.0;
        lock_marker.color.g = 0.0;
        lock_marker.color.b = 0.0;
        lock_marker.color.a = 1.0;
        lock_marker.text = "Locked";
    }
    else if(!rover_lock_mobility_ && !rover_lock_actuation_){
        // Color white 
        lock_marker.color.r = 1.0;
        lock_marker.color.g = 1.0;
        lock_marker.color.b = 1.0;
        lock_marker.color.a = 1.0;
        lock_marker.text = "Unlocked";
    }
    else{
        // Color Yellow
        lock_marker.color.r = 1.0;
        lock_marker.color.g = 1.0;
        lock_marker.color.b = 0.0;
        lock_marker.color.a = 1.0;
        lock_marker.text = "Error";
    }

    visualization_msgs::msg::Marker op_marker;
    op_marker.header.frame_id = "map";
    op_marker.header.stamp = this->get_clock()->now();
    op_marker.ns = "op_mode";
    op_marker.id = 0;
    op_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    op_marker.action = visualization_msgs::msg::Marker::ADD;
    op_marker.pose.position.x = 6.3;
    op_marker.pose.position.y = 6.5;
    op_marker.pose.position.z = 0.0;
    op_marker.pose.orientation.x = 0.0;
    op_marker.pose.orientation.y = 0.0;
    op_marker.pose.orientation.z = 0.0;
    op_marker.pose.orientation.w = 1.0;
    op_marker.scale.x = 0.2;
    op_marker.scale.y = 0.2;
    op_marker.scale.z = 0.2;
    switch(rover_op_mode_){
        case 0:
            // Color green-Standby
            op_marker.color.r = 0.0;
            op_marker.color.g = 1.0;
            op_marker.color.b = 0.0;
            op_marker.color.a = 1.0;
            op_marker.text = "OP-Standby";
            break;
        case 1:
            // Color white-Teleop
            op_marker.color.r = 1.0;
            op_marker.color.g = 1.0;
            op_marker.color.b = 1.0;
            op_marker.color.a = 1.0;
            op_marker.text = "OP-Teleop";
            break;
        case 2:
            // Color Blue-Autonomous
            op_marker.color.r = 0.0;
            op_marker.color.g = 0.0;
            op_marker.color.b = 1.0;
            op_marker.color.a = 1.0;
            op_marker.text = "OP-Autonomous";
            break;
        default:
            // Color Yellow-Error
            op_marker.color.r = 1.0;
            op_marker.color.g = 1.0;
            op_marker.color.b = 0.0;
            op_marker.color.a = 1.0;
            op_marker.text = "OP-Error";
            break;
    }

    visualization_msgs::msg::Marker task_marker;
    task_marker.header.frame_id = "map";
    task_marker.header.stamp = this->get_clock()->now();
    task_marker.ns = "task_mode";
    task_marker.id = 0;
    task_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    task_marker.action = visualization_msgs::msg::Marker::ADD;  
    task_marker.pose.position.x = 6.3;
    task_marker.pose.position.y = 6.2;
    task_marker.pose.position.z = 0.0;
    task_marker.pose.orientation.x = 0.0;
    task_marker.pose.orientation.y = 0.0;
    task_marker.pose.orientation.z = 0.0;
    task_marker.pose.orientation.w = 1.0;
    task_marker.scale.x = 0.2;
    task_marker.scale.y = 0.2;
    task_marker.scale.z = 0.2;
    switch(rover_task_mode_){
        case 0:
            // Color green-Idle
            task_marker.color.r = 0.0;
            task_marker.color.g = 1.0;
            task_marker.color.b = 0.0;
            task_marker.color.a = 1.0;
            task_marker.text = "Task-Idle";
            break;
        case 1:
            // Color white-Navigation
            task_marker.color.r = 1.0;
            task_marker.color.g = 1.0;
            task_marker.color.b = 1.0;
            task_marker.color.a = 1.0;
            task_marker.text = "Task-Navigation";
            break;
        case 2:
            // Color Blue-Excavation
            task_marker.color.r = 0.0;
            task_marker.color.g = 0.0;
            task_marker.color.b = 1.0;
            task_marker.color.a = 1.0;
            task_marker.text = "Task-Excavation";
            break;
        case 3:
            // Color Red-Dumping
            task_marker.color.r = 1.0;
            task_marker.color.g = 0.0;
            task_marker.color.b = 0.0;
            task_marker.color.a = 1.0;
            task_marker.text = "Task-Dumping";
            break;
        default:
            // Color Yellow-Error
            task_marker.color.r = 1.0;
            task_marker.color.g = 1.0;
            task_marker.color.b = 0.0;
            task_marker.color.a = 1.0;
            task_marker.text = "Task-Error";
    }

    msg.markers.push_back(lock_marker);
    msg.markers.push_back(op_marker);
    msg.markers.push_back(task_marker);

    modes_marker_publisher_->publish(msg);
}
