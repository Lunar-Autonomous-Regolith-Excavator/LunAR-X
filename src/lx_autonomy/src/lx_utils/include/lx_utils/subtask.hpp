#ifndef SUB_TASK_H
#define SUB_TASK_H

class SubTask
{
    private:
        // Subtask ID
        unsigned int subtask_id_ = 0;

    public:
        // Functions ----------------------------
        /*
        * Constructor
        * */
        SubTask(unsigned int);

        /*
        * Destructor
        * */
        ~SubTask(){}

        /*
        * Return ID of the subtask
        * */
        unsigned int getID();
        // --------------------------------------

};

class MobilitySubTask : public SubTask
{
    private:
        // TODO Goal

    public:
        // Functions ----------------------------
        /*
        * Constructor
        * */
        MobilitySubTask(unsigned int );

        /*
        * Destructor
        * */
        ~MobilitySubTask(){}

        // --------------------------------------

};

class LinActSubTask : public SubTask
{
    private:
        // TODO Linear actuator height

    public:
        // Functions ----------------------------
        /*
        * Constructor
        * */
        LinActSubTask(unsigned int );

        /*
        * Destructor
        * */
        ~LinActSubTask(){}

        // --------------------------------------

};

class DrumSubTask : public SubTask
{
    private:
        // TODO Drum speed

    public:
        // Functions ----------------------------
        /*
        * Constructor
        * */
        DrumSubTask(unsigned int );

        /*
        * Destructor
        * */
        ~DrumSubTask(){}

        // --------------------------------------

};

#endif