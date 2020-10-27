#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"
#include "SwitchBlockMsg_FS.hpp"
#include "common_srv/IntegerMsg.hpp"

class SwitchTrigger : public FlightElement{

public:
	block_id target_block;
    IntegerMsg int_msg;

    void perform();
    void receiveMsgData(DataMessage* t_msg);
    
    SwitchTrigger();
    ~SwitchTrigger();
};