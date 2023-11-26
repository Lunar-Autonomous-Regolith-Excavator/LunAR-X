/* Author: Vibhakar Mohta
 * Subscribers:
 *    - /rover_hw_cmd: [lx_msgs::msg::RoverCommand] Twist command for Husky, float64 [-1 to 1] for linear actuator, float64 [rad/sec] for drum]
 *    - /drum_raw_current: drum_current_read
 *    - /drum_raw_position: drum_ticks
 *    - /actuator_raw_current: acc_current_read
 * Publishers:
 *    - /cmd_vel: [geometry_msgs::msg::Twist] Husky A200 command
 *    - /drum_cmd: [std_msgs::msg::Int32] Drum PWM command [-255 to 255]
 *    - /acc_cmd: [std_msgs::msg::Int32] Linear Actuator PWM command [-255 to 255]
 *    - /tool_info: [lx_msgs::msg::ToolInfo] tool localization and currents [Published whenever /tool_raw_info is recieved]
 * Actions:
 *    - /WeightEstimate - Server - [WeightEstimate] to estimate amount of excavated material
 *
 * - Enforces Hardware limits on Tool and Rover Actuation
 * - Stores Calibration offsets for linear actuator (saved when the callibration service is called)
 * - Scales drum ticks to position [rad] for drum, ticks to m for linear actuator, current analogRead values to Amps
 * - LOCKS all actuation commands when input not recieved for 3 seconds from /rover_hw_cmd (And updates same in Parameter server)
 *
 * TODO
 * - Add limits to control commands (interface with parameter server)
 * - After lock, update status in parameter server
 * - Change service name to different formatting
 * */

#include "lx_hardware_mux/hardware_mux.hpp"

HardwareMux::HardwareMux(): Node("hardware_mux_node")
{
    // Create subscribers
    rover_hw_cmd_sub_ = this->create_subscription<lx_msgs::msg::RoverCommand>(
        "rover_hw_cmd", 10, std::bind(&HardwareMux::roverHardwareCmdCB, this, std::placeholders::_1));
    drum_raw_current_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "drum_raw_current", 10, std::bind(&HardwareMux::drumCurrentCB, this, std::placeholders::_1));
    drum_raw_position_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "drum_raw_position", 10, std::bind(&HardwareMux::drumPositionCB, this, std::placeholders::_1));
    acc_raw_current_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "actuator_raw_current", 10, std::bind(&HardwareMux::accCurrentCB, this, std::placeholders::_1));
   
    // Create publishers
    husky_node_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    drum_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>("drum_cmd", 10);
    acc_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>("acc_cmd", 10);
    tool_info_pub_ = this->create_publisher<lx_msgs::msg::ToolInfo>("tool_info", 10);

    // Create timers
    rover_lock_timer_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&HardwareMux::roverLockCB, this));
    control_publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&HardwareMux::controlPublishCB, this));
    tool_info_publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&HardwareMux::toolInfoPublishCB, this));

    // Create actions
    this->action_server_ = rclcpp_action::create_server<WeightEstimate>(
      this,
      "WeightEstimate",
      std::bind(&HardwareMux::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&HardwareMux::handle_cancel, this, std::placeholders::_1),
      std::bind(&HardwareMux::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Hardware Mux Initialized");
}

//Subscribes to Rover Command Topic and stores desired control commands in class variables
void HardwareMux::roverHardwareCmdCB(const lx_msgs::msg::RoverCommand::SharedPtr msg)
{
    rover_lock_timer_->reset(); //reset live estop 
    this->rover_locked = false;
    if(this->is_action_running == false) //set control commands only if action is not running, else action will take control
    {
        acc_cmd.data = std::min(255.0, std::max((double)msg->actuator_speed*255.0, -255.0));
        drum_cmd.data = std::min(255.0, std::max((double)msg->drum_speed*255.0, -255.0));
    }
    husky_cmd = msg->mobility_twist;
}

void HardwareMux::toolInfoPublishCB()
{
    tool_info_pub_->publish(this->tool_info_msg);
}

void HardwareMux::drumCurrentCB(const std_msgs::msg::Float64::SharedPtr msg)
{
    // Publish Drum Current
    double drum_current = drum_current_scale*msg->data + drum_current_offset;
    filtered_drum_current = 0.9*filtered_drum_current + 0.1*drum_current;
    this->tool_info_msg.drum_current = filtered_drum_current;
    tool_info_msg_time = std::chrono::system_clock::now();
}

void HardwareMux::accCurrentCB(const std_msgs::msg::Float64::SharedPtr msg)
{
    // Publish Actuator Current
    double acc_current = acc_current_scale*msg->data + acc_current_offset;
    filtered_acc_current = 0.9*filtered_acc_current + 0.1*acc_current;
    this->tool_info_msg.acc_current = filtered_acc_current;
    tool_info_msg_time = std::chrono::system_clock::now();
}

void HardwareMux::drumPositionCB(const std_msgs::msg::Float64::SharedPtr msg)
{
    // Publish Drum Position
    this->tool_info_msg.drum_pos = drum_rps_scale*msg->data;
    tool_info_msg_time = std::chrono::system_clock::now();
}

//Triggered by timer, publishes control commands to Husky and Tools(Arduino)
void HardwareMux::controlPublishCB()
{
    if(this->rover_locked == true) // Rover is locked, publish 0 control commands
    {
        drum_cmd.data = 0; 
        acc_cmd.data = 0;
        husky_cmd.linear.x = 0; husky_cmd.linear.y = 0; husky_cmd.linear.z = 0; 
        husky_cmd.angular.x = 0; husky_cmd.angular.y = 0; husky_cmd.angular.z = 0;
    }
    // Publish Control Commands
    husky_node_pub_->publish(husky_cmd);
    drum_cmd_pub_->publish(drum_cmd);
    acc_cmd_pub_->publish(acc_cmd);
}

void HardwareMux::roverLockCB()
{
    RCLCPP_ERROR(this->get_logger(), "[Hardware Mux] No communication Autonomy Docker, Rover Locked");
    this->rover_locked = true;
}

// Action Server Callbacks
rclcpp_action::GoalResponse HardwareMux::handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const WeightEstimate::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with request %d", goal->request);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse HardwareMux::handle_cancel(const std::shared_ptr<GoalHandleWeightEstimate> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void HardwareMux::handle_accepted(const std::shared_ptr<GoalHandleWeightEstimate> goal_handle)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&HardwareMux::execute, this, _1), goal_handle}.detach();
}

void HardwareMux::execute(const std::shared_ptr<GoalHandleWeightEstimate> goal_handle)
{
    /*TODO: 
    - add movement of linear actuator to highest point before calling action
    - add checks for estop modes and operation modes, and other checks for communications with autonomy container
    */ 
    auto result = std::make_shared<WeightEstimate::Result>();
    if(this->rover_locked == true) // Rover is locked, publish 0 control commands
    {
        result->result = -1;
        goal_handle->abort(result);
        RCLCPP_INFO(this->get_logger(), "WeightEstimate: Rover is locked, cannot execute action");
        this->is_action_running = false;
        this->drum_cmd.data = 0;
        return;
    }

    this->is_action_running = true; // Set flag to indicate action is running (so teleop can't take control)
    RCLCPP_INFO(this->get_logger(), "Executing Weight Estimation");
    rclcpp::Rate loop_rate(10);

    auto feedback = std::make_shared<WeightEstimate::Feedback>();

    // Set acc_cmd and drum_cmd to 0 to stop linear actuator before starting action
    this->acc_cmd.data = 0;
    this->drum_cmd.data = 0;

    // Startup variables
    auto start_time = std::chrono::system_clock::now();
    auto start_drum_ticks = this->tool_info_msg.drum_pos;
    auto integral_current = 0.0;
    double action_ticks = 3000;
    while(rclcpp::ok())
    {
        // 15 second action timeout
        auto curr_time = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = curr_time - start_time;
        if (elapsed_seconds.count() > 15.0) 
        {
            result->result = -1;
            goal_handle->abort(result);
            RCLCPP_INFO(this->get_logger(), "WeightEstimate: timed out");
            this->is_action_running = false;
            this->drum_cmd.data = 0;
            return;
        }

        // If no feedback for 2 seconds then timeout
        if((curr_time - tool_info_msg_time) > std::chrono::seconds(2))
        {
            std::chrono::duration<double> elapsed_seconds = curr_time - tool_info_msg_time;
            std::cout<<elapsed_seconds.count()<<std::endl;
            result->result = -1;
            goal_handle->abort(result);
            RCLCPP_INFO(this->get_logger(), "WeightEstimate: Sensor Reading Timed Out");
            this->is_action_running = false;
            this->drum_cmd.data = 0;
            return;
        }

        auto ticks_rotated = std::abs(this->tool_info_msg.drum_pos - start_drum_ticks);

        // if ticks rotated is greater than action_ticks, then action is complete
        if(ticks_rotated > action_ticks)
        {
            result->result = integral_current;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "WeightEstimate: Succeeded, Integral = %f", integral_current);
            this->is_action_running = false;
            drum_cmd.data = 0; 
            return;
        }
  
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) 
        {
            result->result = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "WeightEstimate Canceled");
            this->is_action_running = false;
            drum_cmd.data = 0;
            return;
        }

        // Give control command to drum
        drum_cmd.data = -100;

        // Publish feedback (time elapsed in seconds)
        feedback->feedback = elapsed_seconds.count();
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback");

        //integrate current (ignore values for first 0.25 seconds)
        if(elapsed_seconds.count() > 0.25) integral_current += std::abs(this->tool_info_msg.drum_current);

        // Sleep for the time remaining to let us hit our 10Hz publish rate
        loop_rate.sleep();
    }
    this->is_action_running = false;
    drum_cmd.data = 0;
}