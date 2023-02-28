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

        /*
        * [Pure Virtual Function]
        * Execute the subtask. See derived definitions
        * */
        virtual bool execute() = 0;
        // --------------------------------------

};

class MobilitySubTask : virtual public SubTask
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

        /*
        *
        * TODO
        * */
        bool execute();

        // --------------------------------------

};

class LinActSubTask : virtual public SubTask
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

        /*
        *
        * TODO
        * */
        bool execute();

        // --------------------------------------

};

class DrumSubTask : virtual public SubTask
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

        /*
        *
        * TODO
        * */
        bool execute();

        // --------------------------------------

};

#endif