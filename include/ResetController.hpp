#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"
class ResetController : public FlightElement{
private:
	
public:
	block_id target_block;
    void perform();

    void receive_msg_data(DataMessage* t_msg);
    
    ResetController();
    ~ResetController();
};