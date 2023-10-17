/* Author: Dhruv Tyagi
 * Subscribers:
 *    - /topic: description
 * Publishers:
 *    - /topic: description
 * Services:
 *    - /name (type): description
 * Actions:
 *    - /name (type): description
 *
 * - Summary
 * 
 * TODO
 * - Add todos
 * */

#include "lx_description/joint_viz.hpp"

JointViz::JointViz(): Node("joint_viz_node"){
    // Set up subscriptions & publishers
    setupCommunications();

    RCLCPP_INFO(this->get_logger(), "joint_viz initialized");
}

void JointViz::setupCommunications(){
    // Subscribers
    tool_height_subscriber_ = this->create_subscription<std_msgs::msg::Float64>("tool_height", 10, 
                        std::bind(&JointViz::toolHeightCallBack, this, std::placeholders::_1));
    // Publishers
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
}

void JointViz::toolHeightCallBack(const std_msgs::msg::Float64::SharedPtr msg){
    // Create joint state message
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = msg->header.stamp;
    joint_state_msg.name = {"lift_assembly_joint"};
    joint_state_msg.position = {(msg->data - 0.096) * 1.622 - 0.3};
    joint_state_msg.velocity = {0};
    joint_state_msg.effort = {0};

    // Publish joint state message
    joint_state_publisher_->publish(joint_state_msg);
}

