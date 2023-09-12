#ifndef OPERATIONS_HANDLER_H
#define OPERATIONS_HANDLER_H

#include <rclcpp/rclcpp.hpp>
#include <queue>
#include <list>
#include <memory>
#include <functional>
#include <thread>
#include "lx_library/task.hpp"
#include "geometry_msgs/msg/point.msg"
#include "lx_msgs/msg/berm_config.msg"
#include "lx_msgs/action/operation.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"


class OperationsHandler: public rclcpp::Node
{
    private:
        using Operation = lx_msgs::action::Operation;
        using GoalHandleOperation = rclcpp_action::ServerGoalHandle<Operation>;
        std::vector<unsigned int> executed_task_ids_ {};

        std::queue<std::shared_ptr<Task>, std::list<std::shared_ptr<Task>>> task_queue_ {};
        lx_msgs::msg::BermConfig berm_config_;

        // Action server
        rclcpp_action::Server<Operation>::SharedPtr operation_action_server_;

        // Functions
        void setupCommunications();
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& , std::shared_ptr<const Operation::Goal> );
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleOperation> );
        void handle_accepted(const std::shared_ptr<GoalHandleOperation> );
        void executeOperation(const std::shared_ptr<GoalHandleOperation> );
        std::queue<std::shared_ptr<Task>, std::list<std::shared_ptr<Task>>> planner();
        bool checkBermBuilt();
        bool executeTaskQueue();

    public:
        OperationsHandler();
        ~OperationsHandler(){}
};

RCLCPP_COMPONENTS_REGISTER_NODE(OperationsHandler)

#endif