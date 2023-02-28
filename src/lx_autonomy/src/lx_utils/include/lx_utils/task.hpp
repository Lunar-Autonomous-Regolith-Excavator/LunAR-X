#ifndef TASK_H
#define TASK_H

#include "lx_utils/subtask.hpp"
#include <queue>
#include <list>

class Task
{
    private:
        unsigned int task_id_ = 0;
        std::queue<SubTask, std::list<SubTask>> subtask_queue {};

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
        bool pushSubTask(SubTask );

        // TODO
        bool popSubTask();

        // --------------------------------------

};

#endif