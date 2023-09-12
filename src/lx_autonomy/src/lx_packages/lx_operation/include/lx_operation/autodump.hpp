#ifndef AUTODUMP_H
#define AUTODUMP_H

#include <rclcpp/rclcpp.hpp>
#include "lx_library/task.hpp"

class AutoDump: public rclcpp::Node
{
    private:

    public:
        AutoDump();
        ~AutoDump(){}
};

#endif