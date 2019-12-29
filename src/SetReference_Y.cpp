#include "SetReference_Y.hpp"

SetReference_Y::SetReference_Y(){}

SetReference_Y::~SetReference_Y(){}

void SetReference_Y::perform(){
    _pose_reference.setPoseY(setpoint_y);
    this->emit_message((DataMessage*)&_pose_reference);
}

void SetReference_Y::receive_msg_data(DataMessage* t_msg){

}



