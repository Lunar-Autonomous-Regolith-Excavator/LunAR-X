#include "lx_library/task.hpp"

Task::Task(unsigned int id, TaskTypeEnum type, geometry_msgs::msg::PoseArray pose_array, lx_msgs::msg::BermSection berm_point){
    this->task_id_ = id;
    this->task_type_ = type;
    this->pose_array_ = pose_array;
    this->berm_point_ = berm_point;
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

lx_msgs::msg::BermSection Task::getBermPoint(){
    return berm_point_;
}