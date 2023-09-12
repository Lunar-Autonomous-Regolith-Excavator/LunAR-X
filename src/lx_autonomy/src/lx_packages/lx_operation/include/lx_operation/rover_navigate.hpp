#ifndef ROVER_NAVIGATE_H
#define ROVER_NAVIGATE_H

#include <rclcpp/rclcpp.hpp>
#include "lx_library/subtask.hpp"
#include "lx_library/task.hpp"

class RoverNavigate: public rclcpp::Node
{
    private:

    public:
        RoverNavigate();
        ~RoverNavigate(){}
};

#endif