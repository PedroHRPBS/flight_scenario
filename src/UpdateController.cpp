#include "UpdateController.hpp"


void UpdateController::perform(){
    ControllerMessage _pid_parameters_message;
    //TODO refactor, remove id from PIDData and send separately
    _pid_parameters_message.setPIDParam(this->pid_data);
    _pid_parameters_message.setMRFTParam(this->mrft_data);
    _pid_parameters_message.setSMParam(this->bb_data);

    this->emitMsgUnicastDefault((DataMessage*)&_pid_parameters_message);
}

void UpdateController::receiveMsgData(DataMessage* t_msg){

}

UpdateController::UpdateController(){

}
UpdateController::~UpdateController(){

}

