#include "SetAbsoluteWaypoint.hpp"

SetAbsoluteWaypoint::SetAbsoluteWaypoint(float t_x, float t_y, float t_z, float t_yaw) {
    _waypoint_x = t_x;
    _waypoint_y = t_y;
    _waypoint_z = t_z;
    _waypoint_yaw = t_yaw;
}

SetAbsoluteWaypoint::~SetAbsoluteWaypoint() {

}

void SetAbsoluteWaypoint::perform(){
    Pose waypoint;
    PosesMsg waypoint_msg;

    waypoint.x = _waypoint_x;

    waypoint.y = _waypoint_y;

    waypoint.z = _waypoint_z;

    waypoint.yaw = _waypoint_yaw;
    
    waypoint_msg.p.poses.push_back(waypoint);

    this->emitMsgUnicastDefault((DataMessage*)&waypoint_msg);
}

void SetAbsoluteWaypoint::receiveMsgData(DataMessage* t_msg){

}