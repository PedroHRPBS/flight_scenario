#include "SwitchToBlock.hpp"


void SwitchToBlock::perform(){
    SwitchToBlockMessage _switch_controller_msg;
    ((MessageToBlock*)&_switch_controller_msg)->target_block=this->target_block;
    this->emit_message((DataMessage*)&_switch_controller_msg);
}

void SwitchToBlock::receive_msg_data(DataMessage* t_msg){

}

SwitchToBlock::SwitchToBlock(){

}
SwitchToBlock::~SwitchToBlock(){

}
