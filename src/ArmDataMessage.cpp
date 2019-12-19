#include "ArmDataMessage.hpp"

msg_type ArmDataMessage::getType(){
    return msg_type::arm_update;
}
const int ArmDataMessage::getSize() {
    return sizeof(*this);
}
ArmDataMessage::ArmDataMessage()
{

}
ArmDataMessage::~ArmDataMessage()
{
    
}
