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
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include <rclcpp/executor.hpp>


class AutoNavHandler: public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        using AutoNav = lx_msgs::action::AutoNav;
        using GoalHandleAutoNav = rclcpp_action::ServerGoalHandle<AutoNav>;
        using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
        using GoalHandleComputePathToPose = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
        using FollowPath = nav2_msgs::action::FollowPath;
        using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;
        // Action blocking
        bool action_blocking_ = false;
        bool action_server_responded_ = false;
        bool action_accepted_ = false;
        bool action_success_ = false;
        // Nav2 ComputePath Results
        nav_msgs::msg::Path path_;
        builtin_interfaces::msg::Duration planning_time_;
        // Nav2 Controller Feedback
        double distance_to_goal_;
        double rov_speed_;
        // Service clients
        rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_params_client_;
        // Action server
        rclcpp_action::Server<AutoNav>::SharedPtr autonav_action_server_;
        // Action clients
        rclcpp_action::Client<ComputePathToPose>::SharedPtr compute_path_client_;
        rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
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

        // Functions to handle Nav2 Planner action
        bool computePath(const geometry_msgs::msg::PoseStamped& );
        
        void computePathResponseCallback(GoalHandleComputePathToPose::SharedPtr );
        
        void computePathResultCallback(const GoalHandleComputePathToPose::WrappedResult& );

        // Functions to handle Nav2 Controller action
        bool followPath();

        void followPathResponseCallback(GoalHandleFollowPath::SharedPtr );

        void followPathFeedbackCallback(GoalHandleFollowPath::SharedPtr , const std::shared_ptr<const FollowPath::Feedback> );

        void followPathResultCallback(const GoalHandleFollowPath::WrappedResult& );

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
};

#endif