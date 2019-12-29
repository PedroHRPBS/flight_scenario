#include "UpdatePoseMessage_FS.hpp"

// UpdatePoseMessage_FS::UpdatePoseMessage_FS(float t_x, float t_y, float t_z, float t_yaw) {
//     _type = msg_type::UPDATEPOSEREFERENCE;
//     _x = t_x;
//     _y = t_y;
//     _z = t_z;
//     _yaw = t_yaw;
// }

UpdatePoseMessage_FS::UpdatePoseMessage_FS() {
    _type = msg_type::UPDATEPOSEREFERENCE;
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
int UpdatePoseMessage_FS::getMask(){
    return _mask;
}
msg_type UpdatePoseMessage_FS::getType(){
    return _type;
}
void UpdatePoseMessage_FS::setPoseMessage(float t_x, float t_y, float t_z, float t_yaw, int t_mask){
    _type = msg_type::UPDATEPOSEREFERENCE;
    _x = t_x;
    _y = t_y;
    _z = t_z;
    _yaw = t_yaw;
    _mask = t_mask;
}

const int UpdatePoseMessage_FS::getSize()
{
    return sizeof(this);
}