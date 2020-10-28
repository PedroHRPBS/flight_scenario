#include "SetAbsoluteWaypoint.hpp"

SetAbsoluteWaypoint::SetAbsoluteWaypoint(float t_x, float t_y, float t_z, float t_yaw) {
    _waypoint_x = t_x;
    _waypoint_y = t_y;
    _waypoint_z = t_z;
    _waypoint_yaw = t_yaw;
    _output_port_0 = new OutputPort(ports_id::OP_0, this);
    _ports = {_output_port_0};
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

    this->_output_port_0->receiveMsgData((DataMessage*)&waypoint_msg);
}