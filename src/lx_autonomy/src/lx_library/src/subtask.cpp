# include "lx_library/subtask.hpp"

SubTask::SubTask(unsigned int id){
    this->subtask_id_ = id;
}

unsigned int SubTask::getID(){
    return this->subtask_id_;
}

MobilitySubTask::MobilitySubTask(unsigned int id): SubTask(id){

}

LinActSubTask::LinActSubTask(unsigned int id): SubTask(id){
    
}

DrumSubTask::DrumSubTask(unsigned int id): SubTask(id){
    
}