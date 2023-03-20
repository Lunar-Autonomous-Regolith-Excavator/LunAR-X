#include "lx_hardware_mux/hardware_mux.hpp"
// Author: Vibhakar Mohta
// Subscribers:
//    - /rover_hw_cmd: Twist command for Husky, float64 [-1 to 1] for linear actuator, float64 [rad/sec] for drum]
//    - /tool_raw_info: Float64MultiArray {drum_dticks_dt, acc_ticks, drum_current_read, acc_current_read}
// Publishers:
//    - /cmd_vel: geometry_msgs/Twist Husky A200 command
//    - /drum_cmd: Int32 Drum PWM command [-255 to 255]
//    - /acc_cmd: Int32 Linear Actuator PWM command [-255 to 255]
//    - /tool_info: lx_msgs/ToolInfo tool localization and currents [Published whenever /tool_raw_info is recieved]
// Services:
//    - Calibrate Linear Actuator Positon
//
// - Enforces Hardware limits on Tool and Rover Actuation
// - Applies PID to control drum speed
// - Stores Calibration offsets for linear actuator (saved when the callibration service is called)
// - Scales dticks/dt to RPM for drum, ticks to m for linear actuator, current analogRead values to Amps
// - LOCKS all actuation commands when input not recieved for 3 seconds from /rover_hw_cmd (And updates same in Parameter server)

//TODO
// - Add a service to calibrate linear actuator position
// - Add limits to control commands (interface with parameter server)
// - After lock, update status in parameter server

HardwareMux::HardwareMux(): Node("hardware_mux_node")
{
    // Create subscribers
    rover_hw_cmd_sub_ = this->create_subscription<lx_msgs::msg::RoverCommand>(
        "rover_hw_cmd", 10, std::bind(&HardwareMux::roverHardwareCmdCB, this, std::placeholders::_1));
    tool_raw_info_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "tool_raw_info", 10, std::bind(&HardwareMux::toolRawInfoCB, this, std::placeholders::_1));
   
    // Create publishers
    husky_node_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    drum_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>("drum_cmd", 10);
    acc_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>("acc_cmd", 10);
    tool_info_pub_ = this->create_publisher<lx_msgs::msg::ToolInfo>("tool_info", 10);

    // Create timers
    rover_lock_timer_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&HardwareMux::roverLockCB, this));
    control_publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&HardwareMux::controlPublishCB, this));

    RCLCPP_INFO(this->get_logger(), "Hardware Mux Initialized");
}

//Subscribes to Rover Command Topic and stores desired control commands in class variables
void HardwareMux::roverHardwareCmdCB(const lx_msgs::msg::RoverCommand::SharedPtr msg)
{
    rover_lock_timer_->reset(); //reset live estop 
    
    drum_des_speed = msg->drum_speed;
    double temp = msg->actuator_speed;
    acc_cmd.data = std::min(255.0, std::max(temp*255, -255.0));
    husky_cmd = msg->mobility_twist;
}

//Subscribes to tool info from Arudino and Publishes Calibrated Tool Info
void HardwareMux::toolRawInfoCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // Publish Tool Info
    lx_msgs::msg::ToolInfo tool_info_msg;
    tool_info_msg.drum_vel = drum_rps_scale*msg->data[0];
    tool_info_msg.acc_pos = acc_rps_scale*msg->data[1] - acc_offset;
    tool_info_msg.drum_current = drum_current_scale*msg->data[2];
    tool_info_msg.acc_current = acc_current_scale*msg->data[3];
    tool_info_pub_->publish(tool_info_msg);

    // Update Drum Speed for PID
    drum_curr_speed = tool_info_msg.drum_vel;
}

//Triggered by timer, publishes control commands to Husky and Tools(Arduino)
void HardwareMux::controlPublishCB()
{
    //Apply PID on msg.drum_speed and drum_curr_speed
    double pid_control = 0;
    std::cout<<"Desired Speed: "<<drum_des_speed<<" Current Speed: "<<drum_curr_speed<<std::endl;
    if(std::abs(drum_des_speed)>1e-3)
    {
        error = drum_des_speed - drum_curr_speed;
        error_integral += error;
        pid_control = Kp*error + Ki*error_integral + Kd*(error - error_prev);
    }
    else
    {
        //Desired speed is 0, reset PID and stop motor
        error = 0;
        error_integral = 0;
        pid_control = 0;
    }
    // pid_control = std::min(255.0, std::max(pid_control, -255.0));
    // drum_cmd.data = pid_control;
    drum_cmd.data = drum_des_speed*(255/0.1);
    //Update error_prev
    error_prev = error;

    // Publish Control Commands
    husky_node_pub_->publish(husky_cmd);
    drum_cmd_pub_->publish(drum_cmd);
    acc_cmd_pub_->publish(acc_cmd);
}

void HardwareMux::roverLockCB()
{
    RCLCPP_ERROR(this->get_logger(), "[Hardware Mux] No communication Autonomy Docker, Rover Locked");
    drum_cmd.data = 0; 
    acc_cmd.data = 0;
    husky_cmd.linear.x = 0; husky_cmd.linear.y = 0; husky_cmd.linear.z = 0; 
    husky_cmd.angular.x = 0; husky_cmd.angular.y = 0; husky_cmd.angular.z = 0;  
}