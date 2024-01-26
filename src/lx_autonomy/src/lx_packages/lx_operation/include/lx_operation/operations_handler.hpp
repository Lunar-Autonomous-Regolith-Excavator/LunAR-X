#ifndef OPERATIONS_HANDLER_H
#define OPERATIONS_HANDLER_H

#include <rclcpp/rclcpp.hpp>
#include <queue>
#include <list>
#include <memory>
#include <functional>
#include <future>
#include <thread>
#include "lx_library/task.hpp"
#include "lx_library/lx_utils.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "lx_msgs/srv/plan.hpp"
#include "lx_msgs/msg/berm_config.hpp"
#include "lx_msgs/action/operation.hpp"
#include "lx_msgs/action/auto_dig.hpp"
#include "lx_msgs/action/auto_dump.hpp"
#include "lx_msgs/action/auto_nav.hpp"
#include "lx_msgs/msg/berm_section.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lx_msgs/msg/node_diagnostics.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "lx_msgs/srv/switch.hpp"
#include "lx_msgs/srv/berm_progress_eval.hpp"
#include "lx_msgs/msg/berm_progress.hpp"

class OperationsHandler: public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        const float BERM_HEIGHT = GLOBAL_BERM_HEIGHT_M;
        const float BERM_SECTION_LENGTH = GLOBAL_BERM_LENGTH_M;
        const float AUTODIG_SPEED = 0.025;
        const float AUTODIG_TIME = 45;
        const int MAX_PLAN_ITERS = 200;
        using Operation = lx_msgs::action::Operation;
        using GoalHandleOperation = rclcpp_action::ServerGoalHandle<Operation>;
        using AutoNav = lx_msgs::action::AutoNav;
        using GoalHandleAutoNav = rclcpp_action::ClientGoalHandle<AutoNav>;
        using AutoDig = lx_msgs::action::AutoDig;
        using GoalHandleAutoDig = rclcpp_action::ClientGoalHandle<AutoDig>;
        using AutoDump = lx_msgs::action::AutoDump;
        using GoalHandleAutoDump = rclcpp_action::ClientGoalHandle<AutoDump>;
        using GeometryPoint = geometry_msgs::msg::Point;
        std::queue<Task, std::list<Task>> task_queue_ {};
        std::queue<Task, std::list<Task>> task_queue_copy_ {};
        lx_msgs::msg::BermConfig berm_config_;
        std::vector<Task> executed_tasks_ {};
        std::vector<lx_msgs::msg::PlannedTask> received_plan_ {};
        unsigned int diagnostic_pub_period_ = 1;
        int current_task_id_ = -1;
        bool first_op_dump_ = true;
        // Action blocking
        bool auto_action_blocking_ = false;
        bool auto_action_server_responded_ = false;
        bool auto_action_accepted_ = false;
        bool auto_action_success_ = false;
        bool planner_blocking_ = false;
        // Time
        float blocking_time_limit_ = 600.0;
        rclcpp::TimerBase::SharedPtr diagnostic_pub_timer_;
        // Publishers
        rclcpp::Publisher<lx_msgs::msg::NodeDiagnostics>::SharedPtr diagnostic_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr plan_viz_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr progress_viz_publisher_;
        // Service clients
        rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_params_client_;
        rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_params_client_;
        rclcpp::Client<lx_msgs::srv::Plan>::SharedPtr planner_client_;
        rclcpp::Client<lx_msgs::srv::Switch>::SharedPtr map_switch_client_;
        rclcpp::Client<lx_msgs::srv::BermProgressEval>::SharedPtr berm_eval_client_;
        // Action server
        rclcpp_action::Server<Operation>::SharedPtr operation_action_server_;
        // Action clients
        rclcpp_action::Client<AutoNav>::SharedPtr auto_nav_action_client_;
        rclcpp_action::Client<AutoDig>::SharedPtr auto_dig_action_client_;
        rclcpp_action::Client<AutoDump>::SharedPtr auto_dump_action_client_;
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
        *   - desired task mode
        * 
        * Publish required rover task mode
        * */
        void switchRoverTaskMode(TaskModeEnum );

        /*
        * Argument(s):
        *   - Goal UUID
        *   - Goal shared pointer
        * 
        * Handle goal request
        * */
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& , std::shared_ptr<const Operation::Goal> );

        /*
        * Argument(s):
        *   - Goal handle shared pointer
        * 
        * Handle action cancel request
        * */
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleOperation> );

        /*
        * Argument(s):
        *   - Goal handle shared pointer
        * 
        * Handle action accepted
        * */
        void handle_accepted(const std::shared_ptr<GoalHandleOperation> );

        /*
        * Argument(s):
        *   - Goal handle shared pointer
        * 
        * Execute requested action
        * */
        void executeOperation(const std::shared_ptr<GoalHandleOperation> );

        /*
        * Get the task plan from planner
        * */
        std::queue<Task, std::list<Task>> getPlan();

        /*
        * Callback function to get task plan
        * */
        void plannerClientCB(rclcpp::Client<lx_msgs::srv::Plan>::SharedFuture );

        /*
        * Execute the received planned tasks
        * */
        bool executeTaskQueue();

        /*
        * Call Autonomous navigation task
        * */
        bool callAutoNav(Task );

        /*
        * Call Autonomous digging task
        * */
        bool callAutoDig(Task );

        /*
        * Call Autonomous deposition task
        * */
        bool callAutoDump(Task );

        /*
        * Response callback function for Navigation
        * */
        void autoNavResponseCB(GoalHandleAutoNav::SharedPtr );

        /*
        * Feedback callback function for Navigation
        * */
        void autoNavFeedbackCB(GoalHandleAutoNav::SharedPtr, const std::shared_ptr<const AutoNav::Feedback> );

        /*
        * Result callback function for Navigation
        * */
        void autoNavResultCB(const GoalHandleAutoNav::WrappedResult& );

        /*
        * Response callback function for Digging
        * */
        void autoDigResponseCB(GoalHandleAutoDig::SharedPtr );

        /*
        * Feedback callback function for Digging
        * */
        void autoDigFeedbackCB(GoalHandleAutoDig::SharedPtr, const std::shared_ptr<const AutoDig::Feedback> );

        /*
        * Result callback function for Digging
        * */
        void autoDigResultCB(const GoalHandleAutoDig::WrappedResult& );

        /*
        * Response callback function for Dumping
        * */
        void autoDumpResponseCB(GoalHandleAutoDump::SharedPtr );

        /*
        * Feedback callback function for Dumping
        * */
        void autoDumpFeedbackCB(GoalHandleAutoDump::SharedPtr, const std::shared_ptr<const AutoDump::Feedback> );

        /*
        * Result callback function for Dumping
        * */
        void autoDumpResultCB(const GoalHandleAutoDump::WrappedResult& );

        /*
        * Diagnostic heartbeat published at a fixed rate
        * */
        void diagnosticPublish();

        /*
        * Create rectangle for visualization of planned dumps
        * */
        std::vector<geometry_msgs::msg::Point> createVizRectangle(float , float , float );

        /*
        * Create visualization for current task to be executed
        * */
        void visualizeCurrentTask(Task );

        /*
        * Update visualization entities on Rviz
        * */
        void visualizationUpdate();

        /*
        * Cleanup visualization entities on Rviz
        * */
        void vizCleanup();

        /*
        * Check if 2 berm segments represent the same dump
        * */
        bool checkSameBermSegment(lx_msgs::msg::BermSection , lx_msgs::msg::BermSection );

        /*
        * Call map switch on/off service
        * */
        void callMapSwitch(bool );

        /*
        * CAll berm evaluation service
        * */
        void callBermEval();

        /*
        * Callback function for berm evaluation
        * */
        void bermEvalCB(rclcpp::Client<lx_msgs::srv::BermProgressEval>::SharedFuture );

        /*
        * Callback function for map switch service
        * */
        void mapSwitchCB(rclcpp::Client<lx_msgs::srv::Switch>::SharedFuture );
        // --------------------------------------

    public:
        // Functions
        /*
        * Constructor
        * */
        explicit OperationsHandler(const rclcpp::NodeOptions&);

        /*
        * Destructor
        * */
        ~OperationsHandler(){}
};

#endif