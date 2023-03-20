# include "lx_library/subtask.hpp"

SubTask::SubTask(unsigned int id){
    this->subtask_id_ = id;
}

unsigned int SubTask::getID(){
    return this->subtask_id_;
}

// Mobility Subtask -----------------------------

MobilitySubTask::MobilitySubTask(unsigned int id): SubTask(id){

}

bool MobilitySubTask::execute(){

}

// ----------------------------------------------

// Linear Actuator Subtask ----------------------

LinActSubTask::LinActSubTask(unsigned int id): SubTask(id){
    
}

bool LinActSubTask::execute(){

}

// ----------------------------------------------

// Drum Subtask ---------------------------------

DrumSubTask::DrumSubTask(unsigned int id): SubTask(id){
    
}

bool DrumSubTask::execute(){

}

// ----------------------------------------------