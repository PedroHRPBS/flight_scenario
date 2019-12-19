#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"

class UpdateReference : public FlightElement{
private:
	
public:
	BlockID target_block;
    void perform(){
        ReferenceMessage _reference_update_msg;
		((ControlMessage*)_reset_controller_msg)->target_block=this->target_block;
        this->emit_message((DataMessage*)&_reset_controller_msg);
    }

    void receive_msg_data(DataMessage* t_msg);
    
    ResetController();
    ~ResetController();
};
