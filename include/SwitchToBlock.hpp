#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"
#include "SwitchToBlockMessage.hpp"
class SwitchToBlock : public FlightElement{
private:
	
public:
	block_id target_block;
    void perform();

    void receive_msg_data(DataMessage* t_msg);
    
    SwitchToBlock();
    ~SwitchToBlock();
};