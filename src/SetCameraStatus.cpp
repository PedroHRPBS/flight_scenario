#include "SetCameraStatus.hpp"

SetCameraStatus::SetCameraStatus(int t_status) {
    _current_status = t_status;
}
SetCameraStatus::~SetCameraStatus() {

}

void SetCameraStatus::perform(){
    IntegerMsg int_msg;
    int_msg.data = _current_status;
    // TODO: FIX
    //this->emitMsgUnicastDefault((DataMessage*)&int_msg);
}