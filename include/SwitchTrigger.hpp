#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"
#include "SwitchBlockMsg_FS.hpp"
#include "common_srv/IntegerMsg.hpp"

class SwitchTrigger : public FlightElement{

private:

    Port* _output_port_0;

public:

    enum ports_id {OP_0};
	block_id target_block;
    IntegerMsg int_msg;

    void perform();
    void process(DataMessage* t_msg, Port* t_port) {};

    SwitchTrigger();
    ~SwitchTrigger();
};