#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"

class UpdatePIDcontroller : public FlightElement{
private:
	
public:
	PID_parameters PIDdata;
	block_id target_block;
    void perform(){
        PIDDataMessage _pid_parameters_message;
        _pid_parameters_message.PIDdata=PIDdata;
        this->emit_message((DataMessage*)&_pid_parameters_message);
    }
	
    void receive_msg_data(DataMessage* t_msg);
    
    UpdatePIDcontroller();
    ~UpdatePIDcontroller();
};
