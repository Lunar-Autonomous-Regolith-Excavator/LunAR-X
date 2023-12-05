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
#include <lx_msgs/msg/tool_info.hpp>
#include <lx_msgs/msg/rover_command.hpp>
#include "lx_msgs/action/auto_dig.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64.hpp"
#include "lx_msgs/msg/node_diagnostics.hpp"
#include "lx_msgs/action/calibrate_imu.hpp"
#include "lx_msgs/srv/pcl_ground_height.hpp"

class AutoDigHandler: public rclcpp::Node
{
    private:
        // IMU calibration action client
        using CalibrateImu = lx_msgs::action::CalibrateImu;
        rclcpp_action::Client<CalibrateImu>::SharedPtr calibrate_imu_action_client_;

        // Variables & pointers -----------------
        using AutoDig = lx_msgs::action::AutoDig;
        using GoalHandleAutoDig = rclcpp_action::ServerGoalHandle<AutoDig>;

        lx_msgs::msg::ToolInfo tool_info_msg_;
        double drum_height_ = 0;
        bool inner_PID_control_rover_ = false;
        unsigned int diagnostic_pub_period_ = 1;
        bool debugging_publish_ = true;
        double curr_ground_height_ = 0;
        // Time
        rclcpp::Time tool_info_msg_time_ = rclcpp::Time(0,0,RCL_ROS_TIME);
        rclcpp::TimerBase::SharedPtr rover_command_timer_;
        rclcpp::TimerBase::SharedPtr diagnostic_pub_timer_;
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
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr drum_height_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pcl_ground_height_sub_;
        // Publishers
        rclcpp::Publisher<lx_msgs::msg::RoverCommand>::SharedPtr rover_auto_cmd_pub_;
        rclcpp::Publisher<lx_msgs::msg::NodeDiagnostics>::SharedPtr diagnostic_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr drum_desired_current_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr drum_current_current_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr drum_desired_height_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr drum_current_height_pub_;

        // Service client to PclGroundHeight service
        rclcpp::Client<lx_msgs::srv::PclGroundHeight>::SharedPtr pcl_ground_height_client_;

        // Autodig Control
        pid_struct autodig_pid_outer_,autodig_pid_inner_;
        double prev_error_current = 0, integral_error_current = 0;
        double prev_error_height = 0, integral_error_height = 0;
        double target_drum_height = -1, target_rover_velocity = 0, target_drum_command = 0;

        // Constants
        const double OUTER_PID_CLIP_MIN = 0.01;
        const double OUTER_PID_CLIP_MAX = 0.25;
        const double FORWARD_SPEED = 0.02; // speed at which the rover moves forward (m/s)
        const double DRUM_COMMAND_EXCAVATION = -0.8; // speed at which the drum rotates [-1, 1], -ve is excavation
        const double NOMINAL_CURRENT_VALUE_I = 1.5; 
        const double NOMINAL_CURRENT_VALUE_F = 3.3;
        const double T_END_SECONDS = 60; // time for which the current is increased from nominal_current_value_i to nominal_current_value_f 
        const double GOTO_TOOL_HEIGHT = 0.24; // the height the tool goes to before starting excavation
        const double END_TOOL_HEIGHT = 0.43; // the height the tool goes to after excavation is complete
        
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

        /*
        * Argument(s):
        *   - Tool info message shared pointer
        * 
        * Callback function for tool info subscription
        * */
        void toolInfoCB(const lx_msgs::msg::ToolInfo::SharedPtr );

        /*
        * Argument(s):
        *   - Drum height message shared pointer
        *   
        * Callback function for drum height subscription
        * */
        void drumHeightCB(const std_msgs::msg::Float64::SharedPtr );

        /*
        * Argument(s):
        * 
        * Publish rover command at a specified rate
        * */
        void roverCommandTimerCallback();

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

        /*
        * Diagnostic heartbeat published at a fixed rate
        * */
        void diagnosticPublish();


        /*
        * Call IMU calibration action
        * */
        void callLocalizationCalibration();

        void pclGroundHeightCB(const std_msgs::msg::Float64::SharedPtr msg);

        void callPclGroundHeightSrv(bool);

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