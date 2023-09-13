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

Task::Task(unsigned int id, TaskTypeEnum type, geometry_msgs::msg::PoseArray pose_array){
    this->task_id_ = id;
    this->task_type_ = type;
    this->pose_array_ = pose_array;
}