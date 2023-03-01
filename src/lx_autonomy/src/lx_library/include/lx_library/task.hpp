#ifndef TASK_H
#define TASK_H

#include "lx_library/subtask.hpp"
#include <queue>
#include <list>
#include <memory>

class Task
{
    private:
        unsigned int task_id_ = 0;
        std::queue<std::shared_ptr<SubTask>, std::list<std::shared_ptr<SubTask>>> subtask_queue {};

    public:
        // Functions ----------------------------
        /*
        * Constructor
        * */
        Task(unsigned int);

        /*
        * Destructor
        * */
        ~Task(){}

        // TODO
        bool pushSubTask(std::shared_ptr<SubTask> );

        // TODO
        bool popSubTask();

        // TODO
        bool executeNextSubTask();

        // --------------------------------------

};

#endif