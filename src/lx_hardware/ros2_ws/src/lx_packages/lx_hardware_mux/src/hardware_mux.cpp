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

    // Create actions
    this->action_server_ = rclcpp_action::create_server<Calibrate>(
      this,
      "calibrate",
      std::bind(&HardwareMux::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&HardwareMux::handle_cancel, this, std::placeholders::_1),
      std::bind(&HardwareMux::handle_accepted, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Hardware Mux Initialized");
}

//Subscribes to Rover Command Topic and stores desired control commands in class variables
void HardwareMux::roverHardwareCmdCB(const lx_msgs::msg::RoverCommand::SharedPtr msg)
{
    rover_lock_timer_->reset(); //reset live estop 
    
    drum_des_speed = msg->drum_speed;
    if(this->is_action_running == false)
    {
        acc_cmd.data = std::min(255.0, std::max((double)msg->actuator_speed*255.0, -255.0));
    }
    husky_cmd = msg->mobility_twist;
}
// add moving average filter to current readings
double filtered_acc_current = 0;
double filtered_drum_current = 0;
//Subscribes to tool info from Arudino and Publishes Calibrated Tool Info
void HardwareMux::toolRawInfoCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // Publish Tool Info
    this->tool_info_msg.drum_vel = drum_rps_scale*msg->data[0];
    this->tool_info_msg.acc_pos = acc_rps_scale*msg->data[1] - acc_offset;
    filtered_drum_current = 0.9*filtered_drum_current + 0.1*msg->data[2];
    filtered_acc_current = 0.9*filtered_acc_current + 0.1*msg->data[3];
    this->tool_info_msg.drum_current = drum_current_scale*filtered_drum_current;
    this->tool_info_msg.acc_current = acc_current_scale*filtered_acc_current;
    tool_info_pub_->publish(this->tool_info_msg);
    tool_info_msg_time = std::chrono::system_clock::now();
    // std::cout<<"Drum Vel: "<<this->tool_info_msg.drum_vel<<" Acc Pos: "<<this->tool_info_msg.acc_pos<<std::endl;
}

//Triggered by timer, publishes control commands to Husky and Tools(Arduino)
void HardwareMux::controlPublishCB()
{
    //Apply PID on msg.drum_speed and drum_curr_speed
    double pid_control = 0;
    double drum_curr_speed = tool_info_msg.drum_vel;
    // std::cout<<"Desired Speed: "<<drum_des_speed<<" Current Speed: "<<drum_curr_speed<<std::endl;
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
    // drum_cmd.data = 0.7*drum_des_speed*(255/0.1);
    if(drum_des_speed>1e-3)
    {
        drum_cmd.data = 200;
    }
    else if(drum_des_speed<-1e-3)
    {
        drum_cmd.data = -200;
    }
    else
    {
        drum_cmd.data = 0;
    }


    // //!!!! this is temporary
    // if(std::abs(acc_cmd.data) > 1)
    // {
    //     acc_cmd.data = (acc_cmd.data>0) ? 200 : -200; //if acc_cmd is positive, set to 200, else set to -200
    // }
    // else 
    // {
    //     acc_cmd.data = 0;
    // }

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
    drum_des_speed = 0; 
    acc_cmd.data = 0;
    husky_cmd.linear.x = 0; husky_cmd.linear.y = 0; husky_cmd.linear.z = 0; 
    husky_cmd.angular.x = 0; husky_cmd.angular.y = 0; husky_cmd.angular.z = 0;  
}

// Action Server Callbacks
rclcpp_action::GoalResponse HardwareMux::handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const Calibrate::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with request %d", goal->request);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse HardwareMux::handle_cancel(const std::shared_ptr<GoalHandleCalibrate> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void HardwareMux::handle_accepted(const std::shared_ptr<GoalHandleCalibrate> goal_handle)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&HardwareMux::execute, this, _1), goal_handle}.detach();
}

void HardwareMux::execute(const std::shared_ptr<GoalHandleCalibrate> goal_handle)
{
    this->is_action_running = true;
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(2);
    auto feedback = std::make_shared<Calibrate::Feedback>();
    auto result = std::make_shared<Calibrate::Result>();
    // ros time in seconds
    auto start_time = std::chrono::system_clock::now();
    auto unique_read_time = std::chrono::system_clock::now();
    auto prev_acc_pos = tool_info_msg.acc_pos;
    while(rclcpp::ok())
    {
        // 10 second action timeout
        auto curr_time = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = curr_time - start_time;
        if (elapsed_seconds.count() > 10.0) 
        {
            result->result = false;
            goal_handle->abort(result);
            RCLCPP_INFO(this->get_logger(), "Calibrate: timed out");
            this->is_action_running = false;
            acc_cmd.data = 0; acc_cmd_pub_->publish(acc_cmd);
            return;
        }

        // If no feedback for 2 seconds then timeout
        if((curr_time - tool_info_msg_time) > std::chrono::seconds(2))
        {
            std::chrono::duration<double> elapsed_seconds = curr_time - tool_info_msg_time;
            std::cout<<elapsed_seconds.count()<<std::endl;
            result->result = false;
            goal_handle->abort(result);
            RCLCPP_INFO(this->get_logger(), "Calibrate: sensor reading timed out");
            this->is_action_running = false;
            acc_cmd.data = 0; acc_cmd_pub_->publish(acc_cmd);
            return;
        }

        // Measure time since last sensor reading change
        std::cout<<"reading "<< this->tool_info_msg.acc_pos<< " " <<prev_acc_pos<<std::endl;
        if(std::abs(this->tool_info_msg.acc_pos - prev_acc_pos)>=std::abs(prev_acc_pos)*0.01)
        {
            std::cout<<"unique reading at time: "<<(curr_time - unique_read_time).count()<<"with reading"<< this->tool_info_msg.acc_pos<< " "
                <<prev_acc_pos<<std::endl;
            unique_read_time = std::chrono::system_clock::now();
            prev_acc_pos = this->tool_info_msg.acc_pos;
        }

        // If no change in sensor reading for 2 seconds then callibrate succeeded
        if((curr_time - unique_read_time) > std::chrono::seconds(2))
        {
            result->result = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Calibrate: succeeded, offest = %f", this->tool_info_msg.acc_pos);
            this->acc_offset = this->tool_info_msg.acc_pos;
            this->is_action_running = false;
            acc_cmd.data = 0; acc_cmd_pub_->publish(acc_cmd);
            return;
        }
  
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) 
        {
            result->result = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Calibrate canceled");
            this->is_action_running = false;
            acc_cmd.data = 0; acc_cmd_pub_->publish(acc_cmd);
            return;
        }

        // Update sequence
        acc_cmd.data = -255;
        acc_cmd_pub_->publish(acc_cmd);

        // Publish feedback (time elapsed in seconds)
        feedback->feedback = elapsed_seconds.count();
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback");

        loop_rate.sleep();
    }
    this->is_action_running = false;

}