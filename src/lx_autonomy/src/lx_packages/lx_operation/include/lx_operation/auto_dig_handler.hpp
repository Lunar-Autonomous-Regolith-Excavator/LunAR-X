#ifndef AUTO_DIG_HANDLER_H
#define AUTO_DIG_HANDLER_H

#include <rclcpp/rclcpp.hpp>
#include <queue>
#include <list>
#include <memory>
#include <functional>
#include <thread>
#include "lx_library/task.hpp"
#include "lx_library/lx_utils.hpp"
#include "lx_msgs/action/auto_dig.hpp"
#include <lx_msgs/msg/tool_info.hpp>
#include <lx_msgs/msg/rover_command.hpp>
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class AutoDigHandler: public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        using AutoDig = lx_msgs::action::AutoDig;
        using GoalHandleAutoDig = rclcpp_action::ServerGoalHandle<AutoDig>;
        // Service clients
        rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_params_client_;
        // Action server
        rclcpp_action::Server<AutoDig>::SharedPtr autodig_action_server_;
        // Parameter handling
        struct lock_struct rover_soft_lock_;
        OpModeEnum current_rover_op_mode_ = OpModeEnum::STANDBY;
        TaskModeEnum current_rover_task_mode_ = TaskModeEnum::IDLE;
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> mob_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> act_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> op_mode_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> task_mode_param_cb_handle_;

        // Subscribers
        rclcpp::Subscription<lx_msgs::msg::ToolInfo>::SharedPtr tool_info_sub_;
        lx_msgs::msg::ToolInfo tool_info_msg_;
        std::chrono::time_point<std::chrono::system_clock> tool_info_msg_time_ = std::chrono::system_clock::now();

        // Publishers
        rclcpp::Publisher<lx_msgs::msg::RoverCommand>::SharedPtr rover_hw_cmd_pub_;

        // Hyperparameters for PID height control
        double kp = 1;
        double ki = 0;
        double kd = 0;
        double nominal_current_value = 1; // value of the drum current for correct operation
        double pid_height_error_threshold_cm = 0.1; // error threshold after which tool height control is called
        double height_control_period_seconds = 0.1; // time for which the tool height is controled once PID height error threshold is reached
        double actuator_control_value = 0.7; // speed to more the linear actuator, range [-1, 1]
        
        // PID variables
        double prev_error = 0, integral_error = 0;
        
        // --------------------------------------

        // Functions ----------------------------
        /*
        * Set up subscribers, publishers, clients & servers of the node
        * */
        void setupCommunications();

        /*
        * Set up tracking of global parameters
        * */
        void setupParams();

        /*
        * Get starting values of global parameters
        * */
        void getParams();

        /*
        * Callback function for starting values of global parameters
        * */
        void paramCB(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture );

        // Subscriber callbacks
        void toolInfoCB(const lx_msgs::msg::ToolInfo::SharedPtr );

        /*
        * Argument(s):
        *   - Goal UUID
        *   - Goal shared pointer
        * 
        * Handle goal request
        * */
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& , std::shared_ptr<const AutoDig::Goal> );

        /*
        * Argument(s):
        *   - Goal handle shared pointer
        * 
        * Handle action cancel request
        * */
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAutoDig> );

        /*
        * Argument(s):
        *   - Goal handle shared pointer
        * 
        * Handle action accepted
        * */
        void handle_accepted(const std::shared_ptr<GoalHandleAutoDig> );

        /*
        * Argument(s):
        *   - Goal handle shared pointer
        * 
        * Execute requested action
        * */
        void executeAutoDig(const std::shared_ptr<GoalHandleAutoDig> );

    public:
        // Functions
        /*
        * Constructor
        * */
        AutoDigHandler(const rclcpp::NodeOptions&);
        
        /*
        * Destructor
        * */
        ~AutoDigHandler(){}
};

#endif