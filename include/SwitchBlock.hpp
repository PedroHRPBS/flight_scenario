#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"
#include "SwitchBlockMsg_FS.hpp"

class SwitchBlock : public FlightElement{

public:
	block_id target_block;
    SwitchBlockMsg_FS switch_msg;

    void perform();
    void receiveMsgData(DataMessage* t_msg);
    
    SwitchBlock();
    ~SwitchBlock();
};