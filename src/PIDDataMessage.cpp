#include "PIDDataMessage.hpp" 
msg_type PIDDataMessage::getType(){
    return msg_type::pid_data_update;
}
const int PIDDataMessage::getSize() {
    return sizeof(*this);
}
PIDDataMessage::PIDDataMessage(){}
PIDDataMessage::~PIDDataMessage(){}
