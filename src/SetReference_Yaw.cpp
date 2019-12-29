#include "SetReference_Yaw.hpp"

SetReference_Yaw::SetReference_Yaw() {

}

SetReference_Yaw::~SetReference_Yaw() {

}
void SetReference_Yaw::perform(){
    _pose_reference.setPoseYaw(setpoint_yaw);
    this->emit_message((DataMessage*)&_pose_reference);
}

void SetReference_Yaw::receive_msg_data(DataMessage* t_msg){

}



