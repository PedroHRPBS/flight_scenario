#include "SetReference_Z.hpp"

SetReference_Z::SetReference_Z() {

}

SetReference_Z::~SetReference_Z() {

}

void SetReference_Z::perform(){
    _pose_reference.setPoseZ(setpoint_z);
    this->emit_message((DataMessage*)&_pose_reference);
}

void SetReference_Z::receive_msg_data(DataMessage* t_msg){

}



