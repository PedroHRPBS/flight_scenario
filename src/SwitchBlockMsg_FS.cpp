#include "SwitchBlockMsg_FS.hpp"

SwitchBlockMsg_FS::SwitchBlockMsg_FS() {

}

SwitchBlockMsg_FS::~SwitchBlockMsg_FS() {

}

msg_type SwitchBlockMsg_FS::getType(){
    return _type;
}

const int SwitchBlockMsg_FS::getSize(){
    return sizeof(this);
}

void SwitchBlockMsg_FS::setSwitchBlockMsg_FS(block_id t_block_out, block_id t_block_in){

    _type = msg_type::SWITCHBLOCK;
    _block_in = t_block_in;
    _block_out = t_block_out;

}

block_id SwitchBlockMsg_FS::getBlockToSwitchIn(){
    return _block_in;
}

block_id SwitchBlockMsg_FS::getBlockToSwitchOut(){
    return _block_out;
}