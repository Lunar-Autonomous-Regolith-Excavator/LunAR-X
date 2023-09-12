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

Task::Task(unsigned int id, TaskTypeEnum type){
    this->task_id_ = id;
    this->task_type_ = type;
}

bool Task::execute(){
    // Add exception handling
    switch(this->task_type_){
        case TaskTypeEnum::NAV:
            // Call Rover navigate action
            break;

        case TaskTypeEnum::AUTODIG:
            // Call Autodig action
            break;

        case TaskTypeEnum::AUTODUMP:
            // Call Autodump action
            break;
            
        default:
            return false;
    }
}