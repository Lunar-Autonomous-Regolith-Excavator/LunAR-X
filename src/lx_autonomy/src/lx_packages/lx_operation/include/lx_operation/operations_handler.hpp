#ifndef OPERATIONS_HANDLER_H
#define OPERATIONS_HANDLER_H

#include <rclcpp/rclcpp.hpp>
#include "lx_library/task.hpp"
#include <queue>
#include <list>
#include <memory>

class OperationsHandler: public rclcpp::Node
{
    private:
        std::queue<std::shared_ptr<Task>, std::list<std::shared_ptr<Task>>> task_queue_ {};

        // Action server

        // Functions
        Task excavationPlanner();
        Task dumpPlanner();
        Task navigationPlanner();

    public:
        OperationsHandler();
        ~OperationsHandler(){}
};

#endif