#include "SetRelativeWaypoint.hpp"

SetRelativeWaypoint::SetRelativeWaypoint(float t_x, float t_y, float t_z, float t_yaw) {
    _waypoint_x = t_x;
    _waypoint_y = t_y;
    _waypoint_z = t_z;
    _waypoint_yaw = t_yaw;
}

SetRelativeWaypoint::~SetRelativeWaypoint() {

}

void SetRelativeWaypoint::perform(){
    Pose waypoint;
    PosesMsg waypoint_msg;

    waypoint.x = _current_x + _waypoint_x;
    waypoint.y = _current_y + _waypoint_y;
    waypoint.z = _current_z + _waypoint_z;
    waypoint.yaw = _current_yaw + _waypoint_yaw;
    waypoint_msg.p.poses.push_back(waypoint);

    this->emitMsgUnicastDefault((DataMessage*)&waypoint_msg);
}

void SetRelativeWaypoint::receiveMsgData(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::POSITION){
        _current_x = ((PositionMsg*) t_msg)->x;
        _current_y = ((PositionMsg*) t_msg)->y;
        _current_z = ((PositionMsg*) t_msg)->z;
    }
    else if(t_msg->getType() == msg_type::VECTOR3D){
        _current_yaw = ((Vector3DMessage*) t_msg)->getData().z;
    }
}