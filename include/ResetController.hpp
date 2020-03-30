#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"
#include "ResetControllerMessage.hpp"
class ResetController : public FlightElement{
private:
	
public:
	block_id target_block;
    void perform();

    void receiveMsgData(DataMessage* t_msg);
    
    ResetController();
    ~ResetController();
};