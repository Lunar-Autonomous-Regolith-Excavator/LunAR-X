/* Author: 
 * Subscribers:
 *    - /topic: description
 * Publishers:
 *    - /topic: description
 * Services:
 *    - /name (type): description
 *
 * - Summary
 * 
 * TODO
 * - Add todos
 * */

#include "lx_library/task.hpp"

Task::Task(unsigned int id, TaskTypeEnum type, geometry_msgs::msg::PoseArray pose_array, geometry_msgs::msg::Point task_point){
    this->task_id_ = id;
    this->task_type_ = type;
    this->pose_array_ = pose_array;
    this->task_point_ = task_point;
}

unsigned int Task::getID(){
    return task_id_;
}

TaskTypeEnum Task::getType(){
    return task_type_;
}

geometry_msgs::msg::PoseArray Task::getPoseArray(){
    return pose_array_;
}

geometry_msgs::msg::Point Task::getTaskPoint(){
    return task_point_;
}