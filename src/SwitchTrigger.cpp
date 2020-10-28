#include "SwitchTrigger.hpp"


void SwitchTrigger::perform() {
    _output_port_0->receiveMsgData((DataMessage*)&int_msg);
}

SwitchTrigger::SwitchTrigger() {
    _output_port_0 = new OutputPort(ports_id::OP_0, this);
    _ports = {_output_port_0};
}
SwitchTrigger::~SwitchTrigger(){
    
}