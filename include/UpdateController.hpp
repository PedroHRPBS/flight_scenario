#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"
#include "ControllerMessage.hpp"
#include "PID_values.hpp"
#include "MRFT_values.hpp"

class UpdateController : public FlightElement{
private:
	
public:
    MRFT_parameters mrft_data;
	PID_parameters pid_data;
	block_id target_block;

    void perform();
    void receive_msg_data(DataMessage* t_msg);
    
    UpdateController();
    ~UpdateController();
};
