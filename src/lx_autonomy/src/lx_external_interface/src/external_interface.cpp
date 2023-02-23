#include "lx_external_interface/external_interface.hpp"

// Constructor
ExternalInterface::ExternalInterface(): Node("external_interface_node"){
    // Lock movement at system start
    rover_soft_lock_.mobility_lock = true;
    rover_soft_lock_.actuation_lock = true;
    
    setupCommunications();

    RCLCPP_INFO(this->get_logger(), "External Interface initialized");
}

// Set up subscription
void ExternalInterface::setupCommunications(){
    // Subscribers
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
                                    "joy", 10 , 
                                    std::bind(&ExternalInterface::joyCallBack, this, std::placeholders::_1));

    // Publishers
    rover_mode_publisher_ = this->create_publisher<lx_msgs::msg::RoverOpMode>("rover_op_mode", 1);
    rover_lock_publisher_ = this->create_publisher<lx_msgs::msg::RoverLock>("rover_lock_status", 10);
    rover_teleop_publisher_ = this->create_publisher<lx_msgs::msg::RoverTeleop>("rover_teleop_cmd", 10);
}

// Joystick subscriber callback
void ExternalInterface::joyCallBack(const sensor_msgs::msg::Joy::SharedPtr joy_msg){

    // Publisher given information to publish rover-op-mode and teleop commands 
    rover_control_pub_thread_ = std::thread(std::bind(&ExternalInterface::roverControlPublish, this, joy_msg));
    // Have to detach thread before it goes out of scope
    rover_control_pub_thread_.detach(); 

}

void ExternalInterface::roverControlPublish(const sensor_msgs::msg::Joy::SharedPtr joy_msg){

    // Publish rover-lock-status
    auto rover_lock_msg = lx_msgs::msg::RoverLock();
    rover_lock_msg.mobility_lock = rover_soft_lock_.mobility_lock;
    rover_lock_msg.actuation_lock = rover_soft_lock_.actuation_lock;
    rover_lock_publisher_->publish(rover_lock_msg);

    // Publish rover-op-mode

    // Publish rover-teleop-cmd

}