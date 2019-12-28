#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"
#include "SwitchBlockMsg.hpp"

class SwitchBlock : public FlightElement{

public:
	block_id target_block;
    SwitchBlockMsg switch_msg;

    void perform();
    void receive_msg_data(DataMessage* t_msg);
    
    SwitchBlock();
    ~SwitchBlock();
};