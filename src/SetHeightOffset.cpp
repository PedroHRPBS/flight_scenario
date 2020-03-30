#include "SetHeightOffset.hpp"

SetHeightOffset::SetHeightOffset() {

}
SetHeightOffset::~SetHeightOffset() {

}

void SetHeightOffset::perform(){
    FloatMsg float_msg;
    float_msg.data = _current_z;
    this->emitMsgUnicastDefault((DataMessage*)&float_msg);
}

void SetHeightOffset::receiveMsgData(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::POSITION){
        _current_z = ((PositionMsg*) t_msg)->z;
    }
}