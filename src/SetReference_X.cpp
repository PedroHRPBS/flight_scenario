#include "SetReference_X.hpp"

SetReference_X::SetReference_X(){}

SetReference_X::~SetReference_X(){}

void SetReference_X::perform(){
    _pose_reference.setPoseX(setpoint_x);
    this->emit_message((DataMessage*)&_pose_reference);
}

void SetReference_X::receive_msg_data(DataMessage* t_msg){

}



