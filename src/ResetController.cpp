
#include "ResetController.hpp"

ResetController::ResetController(){

}
ResetController::~ResetController(){

}

void ResetController::perform(){
    ResetControllerMessage _reset_controller_msg;
    ((MessageToBlock*)&_reset_controller_msg)->target_block=this->target_block;
    
    this->emitMsgUnicastDefault((DataMessage*)&_reset_controller_msg);
}

void ResetController::receiveMsgData(DataMessage* t_msg){

}


