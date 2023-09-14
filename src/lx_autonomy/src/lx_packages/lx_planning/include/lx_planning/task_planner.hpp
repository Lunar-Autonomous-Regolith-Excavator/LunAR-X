#ifndef TASK_PLANNER_H
#define TASK_PLANNER_H

#include "rclcpp/rclcpp.hpp"
#include "lx_library/lx_utils.hpp"
#include "lx_msgs/msg/planned_task.hpp"
#include "lx_msgs/srv/plan.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"


class TaskPlanner: public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        // Subscribers
        
        // Clients
        
        // Publishers

        // --------------------------------------

        // Functions ----------------------------
        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();

        /*
        * Put next function here
        * */
        // --------------------------------------

    public:
        // Functions
        /*
        * Constructor
        * */
        TaskPlanner();

        /*
        * Destructor
        * */
        ~TaskPlanner(){}
};

#endif