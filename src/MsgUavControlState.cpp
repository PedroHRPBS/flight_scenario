#include "MsgUavControlState.hpp"

msg_type MsgUavControlState::getType(){
    return msg_type::UAV_CONTROL_STATE;
}
const int MsgUavControlState::getSize(){
    const int size_msg=4; //TODO Refactor
    return size_msg;
}