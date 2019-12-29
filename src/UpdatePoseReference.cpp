#include "UpdatePoseReference.hpp"

UpdatePoseReference::UpdatePoseReference(){}

UpdatePoseReference::~UpdatePoseReference(){}

void UpdatePoseReference::perform(){
    
    //((MessageToBlock*)&_reference_update_msg)->target_block=this->target_block;
    this->emit_message((DataMessage*)&pose_reference);
}

void UpdatePoseReference::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::POSITION){
        _current_x = ((PositionMsg*) t_msg)->x;
        _current_y = ((PositionMsg*) t_msg)->y;
        _current_z = ((PositionMsg*) t_msg)->z;
        std::cout << "Current X: " << _current_x << std::endl;
    }
    else if(t_msg->getType() == msg_type::VECTOR3D){
        _current_yaw = ((Vector3DMessage*) t_msg)->getData().z;
        //std::cout << "Current Yaw: " << _current_yaw << std::endl;
    }
}



