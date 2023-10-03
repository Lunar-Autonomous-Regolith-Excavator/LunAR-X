#ifndef TASK_PLANNER_H
#define TASK_PLANNER_H

#include "rclcpp/rclcpp.hpp"
#include "lx_library/lx_utils.hpp"
#include "lx_msgs/msg/planned_task.hpp"
#include "lx_msgs/srv/plan.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include <vector>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"


class TaskPlanner: public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        rclcpp::Service <lx_msgs::srv::Plan>::SharedPtr plan_service_server_;

        struct BermSection {
            geometry_msgs::msg::Point center;
            double slope;

            BermSection(geometry_msgs::msg::Point center, double slope){
                this->center = center;
                this->slope = slope;
            }
        };

        std::vector<TaskPlanner::BermSection> berm_sections_;

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
        void taskPlannerCallback(const std::shared_ptr<lx_msgs::srv::Plan::Request> ,
                                      std::shared_ptr<lx_msgs::srv::Plan::Response>);

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