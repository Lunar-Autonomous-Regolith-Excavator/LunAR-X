#ifndef HARDWARE_MUX_H
#define HARDWARE_MUX_H

#include <rclcpp/rclcpp.hpp>
#include <lx_msgs/msg/tool_info.hpp>
#include <lx_msgs/msg/rover_command.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include "std_msgs/msg/string.hpp"
#include "lx_hardware_mux/action/calibrate.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class HardwareMux: public rclcpp::Node
{
    private:
        // Publishers and Subscribers
        rclcpp::Subscription<lx_msgs::msg::RoverCommand>::SharedPtr rover_hw_cmd_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tool_raw_info_sub_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr husky_node_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr drum_cmd_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr acc_cmd_pub_;
        rclcpp::Publisher<lx_msgs::msg::ToolInfo>::SharedPtr tool_info_pub_;

        // Action Server
        using Calibrate = lx_hardware_mux::action::Calibrate;
        using GoalHandleCalibrate = rclcpp_action::ServerGoalHandle<Calibrate>;
        rclcpp_action::Server<Calibrate>::SharedPtr action_server_;

        // Action server callbacks
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Calibrate::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCalibrate> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandleCalibrate> goal_handle);
        void execute(const std::shared_ptr<GoalHandleCalibrate> goal_handle);
        bool is_action_running = false;

        // Callbacks
        void roverHardwareCmdCB(const lx_msgs::msg::RoverCommand::SharedPtr msg);
        void toolRawInfoCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
        void controlPublishCB();
        void roverLockCB();

        //Current msg info
        lx_msgs::msg::ToolInfo tool_info_msg;
        std::chrono::time_point<std::chrono::system_clock> tool_info_msg_time = std::chrono::system_clock::now();

        //Timers
        rclcpp::TimerBase::SharedPtr rover_lock_timer_;
        rclcpp::TimerBase::SharedPtr control_publish_timer_;

        //Variables
        std_msgs::msg::Int32 drum_cmd = std_msgs::msg::Int32(); 
        std_msgs::msg::Int32 acc_cmd = std_msgs::msg::Int32(); 
        geometry_msgs::msg::Twist husky_cmd = geometry_msgs::msg::Twist(); // Husky A200 command intialized to 0        
        
        //Controller Variables
        double Kp =1, Ki = 0.001, Kd = 0.1; // PID gains for drum
        double error = 0, error_integral = 0, error_prev = 0; // PID variables
        double drum_des_speed = 0;

        //Callibation Variables
        double drum_rps_scale = 1; // Scale for drum dticks/dt to rad/s
        double acc_rps_scale = 1; // Scale for linear actuator ticks to m
        double drum_current_scale = 1; // Scale for drum current analogRead to Amps
        double acc_current_scale = 1; // Scale for linear actuator current analogRead to Amps
        double acc_offset = 0; // Offset for linear actuator calibration
        
    public:
        // Functions
        /*
        * Constructor
        * */
        HardwareMux();

        /*
        * Destructor
        * */
        ~HardwareMux(){}
};

#endif