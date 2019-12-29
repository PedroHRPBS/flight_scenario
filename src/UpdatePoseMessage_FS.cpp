#include "UpdatePoseMessage_FS.hpp"

UpdatePoseMessage_FS::UpdatePoseMessage_FS() {
    _type = msg_type::USERREFERENCE;
}

UpdatePoseMessage_FS::~UpdatePoseMessage_FS() {

}

float UpdatePoseMessage_FS::getX(){
    return _x;
}
float UpdatePoseMessage_FS::getY(){
    return _y;
}
float UpdatePoseMessage_FS::getZ(){
    return _z;
}
float UpdatePoseMessage_FS::getYaw(){
    return _yaw;
}
msg_type UpdatePoseMessage_FS::getType(){
    return _type;
}
msg_type_reference UpdatePoseMessage_FS::getRefType(){
    return _ref_type;
}
void UpdatePoseMessage_FS::setPoseMessage(float t_x, float t_y, float t_z, float t_yaw){
    _type = msg_type::USERREFERENCE;
    _x = t_x;
    _y = t_y;
    _z = t_z;
    _yaw = t_yaw;
}

void UpdatePoseMessage_FS::setPoseX(float t_x){
    _ref_type = msg_type_reference::X;
    _x = t_x;
}
void UpdatePoseMessage_FS::setPoseY(float t_y){
    _ref_type = msg_type_reference::Y;
    _y = t_y;
}
void UpdatePoseMessage_FS::setPoseZ(float t_z){
    _ref_type = msg_type_reference::Z;
    _z = t_z;
}
void UpdatePoseMessage_FS::setPoseYaw(float t_yaw){
    _ref_type = msg_type_reference::YAW;
    _yaw = t_yaw;
}

const int UpdatePoseMessage_FS::getSize()
{
    return sizeof(this);
}