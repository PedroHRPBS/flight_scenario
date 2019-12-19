#include "ResetControllerMessage.hpp"

msg_type ResetControllerMessage::getType(){
    return msg_type::RestControllerMessage;
}
const int ResetControllerMessage::getSize() {
    return sizeof(*this);
}
ResetControllerMessage::ResetControllerMessage(){

}
ResetControllerMessage::~ResetControllerMessage(){
    
}
