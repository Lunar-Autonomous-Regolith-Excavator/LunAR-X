#include "lx_external_interface/external_interface.hpp"

ExternalInterface::ExternalInterface(): Node("external_interface_node"){
    // Lock movement at system start
    rover_soft_lock_.mobility_lock = true;
    rover_soft_lock_.actuation_lock = true;
    // Set rover to standby at system start
    current_rover_op_mode_ = OpModeEnum::STANDBY;
    // Timer for active rover lock
    rover_lock_timer_ = this->create_wall_timer(std::chrono::seconds(3), 
                        std::bind(&ExternalInterface::activeLock, this));
    
    // Set up subscriptions & publishers
    setupCommunications();

    // Publish rover lock
    lockRover();

    RCLCPP_INFO(this->get_logger(), "External Interface initialized");
}

void ExternalInterface::setupCommunications(){
    // Subscribers
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10 , 
                            std::bind(&ExternalInterface::joyCallBack, this, std::placeholders::_1));

    // Publishers
    rover_mode_publisher_ = this->create_publisher<lx_msgs::msg::RoverOpMode>("rover_op_mode", 1);
    rover_lock_publisher_ = this->create_publisher<lx_msgs::msg::RoverLock>("rover_lock_status", 10);
    rover_teleop_publisher_ = this->create_publisher<lx_msgs::msg::RoverTeleop>("rover_teleop_cmd", 10);
}

void ExternalInterface::joyCallBack(const sensor_msgs::msg::Joy::SharedPtr joy_msg){

    // Publisher given information to publish rover-op-mode and teleop commands 
    rover_control_pub_thread_ = std::thread(std::bind(&ExternalInterface::roverControlPublish, this, joy_msg));

    // Have to detach thread before it goes out of scope
    rover_control_pub_thread_.detach(); 

}

void ExternalInterface::roverControlPublish(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
    // Active lock timer reset
    rover_lock_timer_.reset();

    // Guide button rising-edge controls locking of actuation & mobility 
    if(joy_msg->buttons[int(JoyButtons::GUIDE)] && !joy_last_state_.buttons[int(JoyButtons::GUIDE)]){
        // Change lock status
        rover_soft_lock_.mobility_lock = !rover_soft_lock_.mobility_lock;
        rover_soft_lock_.actuation_lock = !rover_soft_lock_.actuation_lock;
        // Publish lock status change
        switchRoverLockStatus();
    }

    // Start button rising-edge cycles through the operating modes of the rover
    if(joy_msg->buttons[int(JoyButtons::START)] && !joy_last_state_.buttons[int(JoyButtons::START)]){
        switch(current_rover_op_mode_){
            case OpModeEnum::STANDBY:
                current_rover_op_mode_ = OpModeEnum::TELEOP;
            break;

            case OpModeEnum::TELEOP:
                current_rover_op_mode_ = OpModeEnum::AUTONOMOUS;
            break;

            case OpModeEnum::AUTONOMOUS:
                current_rover_op_mode_ = OpModeEnum::STANDBY;
            break;

            default:
                current_rover_op_mode_ = OpModeEnum::STANDBY;
                RCLCPP_ERROR(this->get_logger(), "Current rover mode invalid, setting to STANDBY");
        }
        switchRoverOpMode();
    }

    // Pass through rover teleop commands
    passRoverTeleopCmd(joy_msg);

    // Store last received joystick state
    setLastJoyState(joy_msg);

}

void ExternalInterface::switchRoverLockStatus(){
    // Publish lock status
    auto rover_lock_msg = lx_msgs::msg::RoverLock();
    rover_lock_msg.mobility_lock = rover_soft_lock_.mobility_lock;
    rover_lock_msg.actuation_lock = rover_soft_lock_.actuation_lock;
    rover_lock_publisher_->publish(rover_lock_msg);

    // Display lock status
    std::string mob_display,act_display;
    rover_soft_lock_.mobility_lock? (mob_display = "LOCKED") : (mob_display = "UNLOCKED");
    rover_soft_lock_.actuation_lock? (act_display = "LOCKED") : (act_display = "UNLOCKED");
    RCLCPP_WARN(this->get_logger(), "Rover lock status: Mobility %s, Actuation %s", mob_display, act_display);
}

void ExternalInterface::lockRover(){
    rover_soft_lock_.mobility_lock = true;
    rover_soft_lock_.actuation_lock = true;
    switchRoverLockStatus();
}

void ExternalInterface::unlockRover(){
    rover_soft_lock_.mobility_lock = false;
    rover_soft_lock_.actuation_lock = false;
    switchRoverLockStatus();
}

void ExternalInterface::switchRoverOpMode(){
    // Publish operation mode
    auto rover_op_mode_msg = lx_msgs::msg::RoverOpMode();
    rover_op_mode_msg.data = uint16_t(current_rover_op_mode_);
    rover_mode_publisher_->publish(rover_op_mode_msg);

    // Display operation mode
    std::string display_string;
    switch(current_rover_op_mode_){
        case OpModeEnum::STANDBY:
            display_string = "STANDBY";
        break;
        case OpModeEnum::TELEOP:
            display_string = "TELEOP";
        break;
        case OpModeEnum::AUTONOMOUS:
            display_string = "AUTONOMOUS";
        break;
    }
    RCLCPP_WARN(this->get_logger(), "Rover operation mode set to %s", display_string);
}

void ExternalInterface::setLastJoyState(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
    joy_last_state_ = *joy_msg;
}

void ExternalInterface::activeLock(){
    if(!rover_soft_lock_.mobility_lock || !rover_soft_lock_.actuation_lock){
        // Lock rover if no /joy message received for 3 seconds
        lockRover();
        // Set to standby
        current_rover_op_mode_ = OpModeEnum::STANDBY;
        switchRoverOpMode();
        RCLCPP_WARN(this->get_logger(), "Lost communication with joystick/control station");
    }
}

void ExternalInterface::passRoverTeleopCmd(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
    // TODO
}