#include "lx_utils/task.hpp"

Task::Task(unsigned int id){
    this->task_id_ = id;
}

bool Task::pushSubTask(SubTask new_sub_task){
    subtask_queue.push(new_sub_task);
    return true;
}

bool Task::popSubTask(){
    subtask_queue.pop();
    return true;
}