#ifndef OPERATIONS_HANDLER_H
#define OPERATIONS_HANDLER_H

#include <rclcpp/rclcpp.hpp>
#include "lx_library/subtask.hpp"
#include "lx_library/task.hpp"

class OperationsHandler: public rclcpp::Node
{
    private:

    public:
        OperationsHandler();
        ~OperationsHandler(){}
};

#endif