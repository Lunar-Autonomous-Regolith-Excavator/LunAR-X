#ifndef AUTO_NAV_HANDLER_H
#define AUTO_NAV_HANDLER_H

#include <rclcpp/rclcpp.hpp>
#include <queue>
#include <list>
#include <memory>
#include <functional>
#include <thread>
#include "lx_library/task.hpp"
#include "lx_library/lx_utils.hpp"
#include "lx_msgs/action/auto_nav.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lx_msgs/msg/rover_command.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class AutoNavHandler: public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        using AutoNav = lx_msgs::action::AutoNav;
        using GoalHandleAutoNav = rclcpp_action::ServerGoalHandle<AutoNav>;
        using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
        using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
        // Action blocking
        bool action_blocking_ = false;
        bool action_server_responded_ = false;
        bool action_accepted_ = false;
        bool action_success_ = false;
        // Goal Pose
        geometry_msgs::msg::PoseStamped goal_pose_;
        int next_action_;
        // Rover Current State
        nav_msgs::msg::Odometry rover_current_pose_;
        geometry_msgs::msg::Twist rover_cmd_vel_;
        // Nav2 Action Feedback
        geometry_msgs::msg::PoseStamped nav2_current_pose_;
        builtin_interfaces::msg::Duration navigation_time_;
        builtin_interfaces::msg::Duration estimated_time_remaining_;
        int number_of_recoveries_;
        double distance_remaining_;

        // Subscriber for rover current pose
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rover_current_pose_sub_;
        // Subscriber for cmd_vel
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        // Subscriber for cmd_vel_nav
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_nav_sub_;
        // Publisher for rover auto command
        rclcpp::Publisher<lx_msgs::msg::RoverCommand>::SharedPtr rover_cmd_pub_;
        // Service clients
        rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_params_client_;
        // Action server
        rclcpp_action::Server<AutoNav>::SharedPtr autonav_action_server_;
        // Action clients
        rclcpp_action::Client<NavigateThroughPoses>::SharedPtr navigate_through_poses_client_;
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
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& , std::shared_ptr<const AutoNav::Goal> );

        /*
        * Argument(s):
        *   - Goal handle shared pointer
        * 
        * Handle action cancel request
        * */
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAutoNav> );

        /*
        * Argument(s):
        *   - Goal handle shared pointer
        * 
        * Handle action accepted
        * */
        void handle_accepted(const std::shared_ptr<GoalHandleAutoNav> );

        /*
        * Argument(s):
        *   - Goal handle shared pointer
        * 
        * Execute requested action
        * */
        void executeAutoNav(const std::shared_ptr<GoalHandleAutoNav> );

        // Functions to handle Nav2 Navigate to Pose action
        bool navigateThroughPoses();

        void navigateThroughPosesResponseCallback(GoalHandleNavigateThroughPoses::SharedPtr );

        void navigateThroughPosesFeedbackCallback(GoalHandleNavigateThroughPoses::SharedPtr , const std::shared_ptr<const NavigateThroughPoses::Feedback> );

        void navigateThroughPosesResultCallback(const GoalHandleNavigateThroughPoses::WrappedResult& );

        // Function to remap cmd_vel_nav to rover_cmd
        void cmdVelNavCallback(const geometry_msgs::msg::Twist::SharedPtr );

        // Functions for rover state subscriber callbacks
        void roverCurrentPoseCallback(const nav_msgs::msg::Odometry::SharedPtr );

        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr );

    public:
        // Functions
        /*
        * Constructor
        * */
        AutoNavHandler(const rclcpp::NodeOptions&);

        /*
        * Destructor
        * */
        ~AutoNavHandler(){}

        // Constants
        static constexpr double MAX_DURATION = 180.0; // seconds
        static constexpr double INTERMEDIATE_GOAL_DISTANCE = 0.3; // meters

        // PID Parameters for Yaw Control
        static constexpr double YAW_TOLERANCE = 2 * M_PI / 180; // radians
        static constexpr double YAW_KP = 15;
        static constexpr double YAW_KI = 0.001;
        static constexpr double YAW_KD = 1;
        static constexpr double YAW_VEL_MAX = 0.1; // radians per second
};

#endif