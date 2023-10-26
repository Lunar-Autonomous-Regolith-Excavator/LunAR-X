/* Author: Dhruv Tyagi
 * Subscribers:
 *    - /tool_height (std_msgs::msg::Float64): Tool height from aruco markers
 * Publishers:
 *    - /joint_states (sensor_msgs::msg::JointState): Joint states for rviz/foxglove
 *
 * - Node to help rviz/foxglove visualization of indirect joint states
 * 
 * TODO
 * - Add drum rotation joint
 * */

#include "lx_description/joint_viz.hpp"

JointViz::JointViz(): Node("joint_viz_node"){
    // Set up subscriptions & publishers
    setupCommunications();

    // Sleep 5 seconds to allow transforms to be published
    rclcpp::sleep_for(std::chrono::seconds(5));

    // Publish initial joint message
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->get_clock()->now();
    joint_state_msg.name = {"lift_assembly_joint"};
    joint_state_msg.position = {-((0.25 - 0.096) * 1.622 - 0.3)};
    joint_state_msg.velocity = {0};
    joint_state_msg.effort = {0};
    joint_state_publisher_->publish(joint_state_msg);

    RCLCPP_INFO(this->get_logger(), "joint_viz initialized");
}

void JointViz::setupCommunications(){
    // Subscribers
    tool_height_subscriber_ = this->create_subscription<std_msgs::msg::Float64>("tool_height", 10, 
                        std::bind(&JointViz::toolHeightCallBack, this, std::placeholders::_1));
    tool_info_subscriber_ = this->create_subscription<std_msgs::msg::Float64>("tool_info", 10, 
                        std::bind(&JointViz::drumRotationCallBack, this, std::placeholders::_1));

    // Publishers
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
}

void JointViz::toolHeightCallBack(const std_msgs::msg::Float64::SharedPtr msg){
    // Create joint state message
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->get_clock()->now();
    joint_state_msg.name = {"lift_assembly_joint"};
    joint_state_msg.position = {-((msg->data - 0.096) * 1.622 - 0.3)};
    joint_state_msg.velocity = {0};
    joint_state_msg.effort = {0};

    // Publish joint state message
    joint_state_publisher_->publish(joint_state_msg);
}

void JointViz::toolInfoCallBack(const lx_msgs::msg::ToolInfo::SharedPtr msg){

    // Drum angle
    double drum_angle = (msg->drum_pos) * twoPi / 3800;

    // Wrap the angle to the [0, 2π] range
    while (drum_angle < 0) {
        drum_angle += twoPi;
    }
    while (drum_angle >= twoPi) {
        drum_angle -= twoPi;
    }

    // Create joint state message
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->get_clock()->now();
    joint_state_msg.name = {"drum_joint"};
    joint_state_msg.position = {drum_angle};
    joint_state_msg.velocity = {0};
    joint_state_msg.effort = {0};

    // Publish joint state message
    joint_state_publisher_->publish(joint_state_msg);
}

