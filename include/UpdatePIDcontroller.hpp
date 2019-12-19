#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"
#include "PIDDataMessage.hpp"

class UpdatePIDcontroller : public FlightElement{
private:
	
public:
	PID_parameters PIDdata;
	block_id target_block;
    void perform();
	
    void receive_msg_data(DataMessage* t_msg);
    
    UpdatePIDcontroller();
    ~UpdatePIDcontroller();
};
