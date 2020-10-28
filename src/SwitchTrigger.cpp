#include "SwitchTrigger.hpp"


void SwitchTrigger::perform(){

    this->emitMsgUnicastDefault((DataMessage*)&int_msg);
}

void SwitchTrigger::receiveMsgData(DataMessage* t_msg){

}

SwitchTrigger::SwitchTrigger(){

}
SwitchTrigger::~SwitchTrigger(){

}
