#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"
#include "ResetControllerMessage.hpp"
class ResetController : public FlightElement{
private:
	Port* _output_port_0;
public:
    enum ports_id {OP_0};
    void process(DataMessage* t_msg, Port* t_port) {};
	block_id target_block;
    void perform();
    
    ResetController();
    ~ResetController();
};