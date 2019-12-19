#include "SwitchToBlockMessage.hpp"

msg_type SwitchToBlockMessage::getType(){
    return msg_type::SwitchToBlock;
}
const int SwitchToBlockMessage::getSize() {
    return sizeof(*this);
}
SwitchToBlockMessage::SwitchToBlockMessage(){

}
SwitchToBlockMessage::~SwitchToBlockMessage(){
    
}
