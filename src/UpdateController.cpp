#include "UpdateController.hpp"


void UpdateController::perform(){
    ControllerMessage _pid_parameters_message;
    //TODO refactor, remove id from PIDData and send separately
    _pid_parameters_message.setPIDParam(this->pid_data);
    _pid_parameters_message.setMRFTParam(this->mrft_data);

    this->emit_message((DataMessage*)&_pid_parameters_message);
}

void UpdateController::receive_msg_data(DataMessage* t_msg){

}

UpdateController::UpdateController(){

}
UpdateController::~UpdateController(){

}

