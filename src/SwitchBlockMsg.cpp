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

void SwitchBlockMsg::setSwitchBlockMsg(block_id t_block_out, block_id t_block_in){

    _type = msg_type::SWITCHBLOCK;
    _block_in = t_block_in;
    _block_out = t_block_out;

}

block_id SwitchBlockMsg::getBlockToSwitchIn(){
    return _block_in;
}

block_id SwitchBlockMsg::getBlockToSwitchOut(){
    return _block_out;
}