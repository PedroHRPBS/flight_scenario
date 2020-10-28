#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"
#include "ControllerMessage.hpp"
#include "PID_values.hpp"
#include "MRFT_values.hpp"
#include "BB_values.hpp"

class UpdateController : public FlightElement{
private:

	Port* _output_port_0;
    
public:

    enum ports_id {OP_0};
    MRFT_parameters mrft_data;
	PID_parameters pid_data;
    BB_parameters bb_data;
	block_id target_block;

    void perform();
    void process(DataMessage* t_msg, Port* t_port) {};
    
    UpdateController();
    ~UpdateController();
};
