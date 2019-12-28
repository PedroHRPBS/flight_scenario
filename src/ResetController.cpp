
#include "ResetController.hpp"

ResetController::ResetController(){

}
ResetController::~ResetController(){

}

void ResetController::perform(){
    ResetControllerMessage _reset_controller_msg;
    ((MessageToBlock*)&_reset_controller_msg)->target_block=this->target_block;
    
    this->emit_message((DataMessage*)&_reset_controller_msg);
}

void ResetController::receive_msg_data(DataMessage* t_msg){

}


