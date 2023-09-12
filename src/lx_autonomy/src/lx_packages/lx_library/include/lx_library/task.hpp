#ifndef TASK_H
#define TASK_H

#include "lx_library/lx_utils.hpp"

class Task
{
    private:
        unsigned int task_id_ = 0;
        TaskTypeEnum task_type_;

    public:
        // Functions ----------------------------
        /*
        * Constructor
        * */
        Task(unsigned int, TaskTypeEnum);

        /*
        * Destructor
        * */
        ~Task(){}

        /*
        * Execute task
        * */
        bool executeTask();

        // --------------------------------------

};

#endif