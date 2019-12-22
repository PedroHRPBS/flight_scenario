#include "UpdatePIDcontroller.hpp"


void UpdatePIDcontroller::perform(){
    PIDDataMessage _pid_parameters_message;
    //TODO refactor, remove id from PIDData and send separately
    _pid_parameters_message.PIDdata=this->PIDdata;
    //_pid_parameters_message.target_block=this->target_block;
    this->emit_message((DataMessage*)&_pid_parameters_message);
}

void UpdatePIDcontroller::receive_msg_data(DataMessage* t_msg){

}

UpdatePIDcontroller::UpdatePIDcontroller(){

}
UpdatePIDcontroller::~UpdatePIDcontroller(){

}

