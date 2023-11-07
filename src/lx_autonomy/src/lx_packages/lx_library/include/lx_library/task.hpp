#ifndef TASK_H
#define TASK_H

#include <rclcpp/rclcpp.hpp>
#include "lx_library/lx_utils.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>

class Task
{
    private:
        unsigned int task_id_;
        TaskTypeEnum task_type_;
        geometry_msgs::msg::PoseArray pose_array_ = geometry_msgs::msg::PoseArray();
        geometry_msgs::msg::Point task_point_ = geometry_msgs::msg::Point();

    public:
        // Functions ----------------------------
        unsigned int getID();

        TaskTypeEnum getType();

        geometry_msgs::msg::PoseArray getPoseArray();

        geometry_msgs::msg::Point getTaskPoint();

        /*
        * Constructor
        * */
        Task(unsigned int, TaskTypeEnum, geometry_msgs::msg::PoseArray, geometry_msgs::msg::Point);

        /*
        * Destructor
        * */
        ~Task(){}
        // --------------------------------------

};

#endif