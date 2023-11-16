#ifndef AUTO_DUMP_HANDLER_H
#define AUTO_DUMP_HANDLER_H

#include <rclcpp/rclcpp.hpp>
#include <queue>
#include <list>
#include <memory>
#include <functional>
#include <thread>
#include "lx_library/task.hpp"
#include "lx_library/lx_utils.hpp"
#include "lx_msgs/action/auto_dump.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <lx_msgs/msg/rover_command.hpp>
#include "lx_msgs/action/auto_dig.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "lx_msgs/srv/switch.hpp"


// Class for PID
class PID{
    public:
        double kp, ki, kd;
        double CLIP_MIN, CLIP_MAX;
        double prev_error = 0, integral_error = 0;
        PID(pid_struct gains, double CLIP_MIN, double CLIP_MAX){
            this->kp = gains.kp;
            this->ki = gains.ki;
            this->kd = gains.kd;
            this->CLIP_MIN = CLIP_MIN;
            this->CLIP_MAX = CLIP_MAX;
        }
        double getCommand(double error)
        {
            double pid_command = kp*error + ki*integral_error + kd*(error - prev_error);
            prev_error = error;
            integral_error += error;
            pid_command = std::min(std::max(pid_command, CLIP_MIN), CLIP_MAX);
            return pid_command;
        }
};


class AutoDumpHandler: public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        using AutoDump = lx_msgs::action::AutoDump;
        using GoalHandleAutoDump = rclcpp_action::ServerGoalHandle<AutoDump>;
        pid_struct tool_height_pid_, rover_x_pid_, rover_yaw_pid_;
        geometry_msgs::msg::Point visual_servo_error_;
        double drum_height_;
        rclcpp::Time servoing_msg_time= rclcpp::Time(0,0,RCL_ROS_TIME);
        rclcpp::Time last_drum_height_msg_time = rclcpp::Time(0,0,RCL_ROS_TIME);
        bool visual_servo_switch_ = false;

        const double DRUM_DUMP_SPEED = 0.8;
        const double DRUM_DUMP_TIME_S = 20;
        const double END_TOOL_HEIGHT = 0.45;
        const double CLIP_VEL_CMD_VAL = 0.05;
        const double CLIP_HEIGHT_CMD_VAL = 1;
        const double CLIP_YAW_CMD_VAL = 0.05;
        
        // Service clients
        rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_params_client_;
        rclcpp::Client<lx_msgs::srv::Switch>::SharedPtr visual_servo_client_;
        
        // Action server
        rclcpp_action::Server<AutoDump>::SharedPtr autodump_action_server_;
        
        // Subscribers
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr drum_height_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr visual_servo_error_sub_;

        // Publishers
        rclcpp::Publisher<lx_msgs::msg::RoverCommand>::SharedPtr rover_auto_cmd_pub_;

        // Parameter handling
        struct lock_struct rover_soft_lock_;
        OpModeEnum current_rover_op_mode_ = OpModeEnum::STANDBY;
        TaskModeEnum current_rover_task_mode_ = TaskModeEnum::IDLE;
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> mob_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> act_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> op_mode_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> task_mode_param_cb_handle_;
        // --------------------------------------

        // Functions ----------------------------
        /*
        * Set up subscribers, publishers, clients & servers of the node
        * */
        void setupCommunications();

        /*
        * 
        * */
        void drumHeightCB(const std_msgs::msg::Float64::SharedPtr);

        /*
        * 
        * */
        void visualServoErrorCB(const geometry_msgs::msg::Point::SharedPtr msg);

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

        /*
        * Argument(s):
        *   - Goal UUID
        *   - Goal shared pointer
        * 
        * Handle goal request
        * */
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& , std::shared_ptr<const AutoDump::Goal> );

        /*
        * Argument(s):
        *   - Goal handle shared pointer
        * 
        * Handle action cancel request
        * */
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAutoDump> );

        /*
        * Argument(s):
        *   - Goal handle shared pointer
        * 
        * Handle action accepted
        * */
        void handle_accepted(const std::shared_ptr<GoalHandleAutoDump> );

        /*
        * Argument(s):
        *   - Goal handle shared pointer
        * 
        * Execute requested action
        * */
        void executeAutoDump(const std::shared_ptr<GoalHandleAutoDump> );

        /*
        * 
        * */
        bool callVisualServoSwitch(bool switch_state, 
                           const lx_msgs::msg::BermSection::SharedPtr current_berm_segment = nullptr,
                           const lx_msgs::msg::BermSection::SharedPtr prev_berm_segment = nullptr);

        /*
        * 
        * */
        void visualServoSwitchCB(rclcpp::Client<lx_msgs::srv::Switch>::SharedFuture );

    public:
        // Functions
        /*
        * Constructor
        * */
        AutoDumpHandler(const rclcpp::NodeOptions&);

        /*
        * Destructor
        * */
        ~AutoDumpHandler(){}
};

#endif