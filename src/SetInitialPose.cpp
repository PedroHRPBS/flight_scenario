#include "SetInitialPose.hpp"

SetInitialPose::SetInitialPose(){}

SetInitialPose::~SetInitialPose(){}

void SetInitialPose::perform(){
    std::cout << "POSE X: "<<_current_x<<std::endl;
    std::cout << "POSE Y: "<<_current_y<<std::endl;
    std::cout << "POSE Z: "<<_current_z<<std::endl;
    std::cout << "POSE Yaw: "<<_current_yaw<<std::endl;

    pose_reference.setPoseX(_current_x);
    this->emit_message((DataMessage*)&pose_reference);
    pose_reference.setPoseY(_current_y);
    this->emit_message((DataMessage*)&pose_reference);
    pose_reference.setPoseZ(_current_z);
    this->emit_message((DataMessage*)&pose_reference);
    pose_reference.setPoseYaw(_current_yaw);
    this->emit_message((DataMessage*)&pose_reference);
}

void SetInitialPose::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::POSITION){
        _current_x = ((PositionMsg*) t_msg)->x;
        _current_y = ((PositionMsg*) t_msg)->y;
        _current_z = ((PositionMsg*) t_msg)->z;
        //std::cout << "POSE X: "<<_current_x<<std::endl;
    }
    else if(t_msg->getType() == msg_type::VECTOR3D){
        _current_yaw = ((Vector3DMessage*) t_msg)->getData().z;
        //std::cout << "POSE yaw: "<<_current_yaw<<std::endl;
    }
}



