#include "lx_library/task.hpp"

Task::Task(unsigned int id){
    this->task_id_ = id;
}

bool Task::pushSubTask(std::shared_ptr<SubTask> new_sub_task){
    subtask_queue.push(new_sub_task);
    return true;
}

bool Task::popSubTask(){
    subtask_queue.pop();
    return true;
}

bool Task::executeNextSubTask(){
    // Add exception handling
    // TODO
    return subtask_queue.front()->execute();
}