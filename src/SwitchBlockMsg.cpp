#include "SwitchBlockMsg.hpp"

SwitchBlockMsg::SwitchBlockMsg() {

}

SwitchBlockMsg::~SwitchBlockMsg() {

}

msg_type SwitchBlockMsg::getType(){
    return _type;
}

const int SwitchBlockMsg::getSize(){
    return sizeof(this);
}

void SwitchBlockMsg::setSwitchBlockMsg(int t_block_in, int t_block_out){

    _type = msg_type::SWITCHBLOCK;
    _block_in = t_block_in;
    _block_out = t_block_out;

}

int SwitchBlockMsg::getBlockToSwitchIn(){
    return _block_in;
}

int SwitchBlockMsg::getBlockToSwitchOut(){
    return _block_out;
}