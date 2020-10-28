#include "SetRelativeWaypoint.hpp"

SetRelativeWaypoint::SetRelativeWaypoint(float t_x, float t_y, float t_z, float t_yaw) {
    _input_port_0 = new InputPort(ports_id::IP_0, this);
    _input_port_1 = new InputPort(ports_id::IP_1, this);
    _output_port_0 = new OutputPort(ports_id::OP_0, this);
    _ports = {_input_port_0, _input_port_1, _output_port_0};
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


    if(_waypoint_x == -10){
        waypoint.x = 0.0;
    }else{
        waypoint.x = _current_x + _waypoint_x;
    }

    if(_waypoint_y == -10){
        waypoint.y = 0.0;
    }else{
        waypoint.y = _current_y + _waypoint_y;
    }

    if(_waypoint_z == -10){
        waypoint.z = 0.0;
    }else if(_waypoint_z == -99){ //TODO fix this, this is just for the MRFT ID
        waypoint.z = 1.0;
    }
    else{
        waypoint.z = _current_z + _waypoint_z;
    }

    if(_waypoint_yaw == -10){
        waypoint.yaw = 0.0;
    }else{
        waypoint.yaw = _current_yaw + _waypoint_yaw;
    }
    
    waypoint_msg.p.poses.push_back(waypoint);

    this->_output_port_0->receiveMsgData((DataMessage*)&waypoint_msg);
}

void SetRelativeWaypoint::process(DataMessage* t_msg, Port* t_port) {
    if(t_port->getID() == ports_id::IP_0) {
        _current_x = ((PositionMsg*) t_msg)->x;
        _current_y = ((PositionMsg*) t_msg)->y;
        _current_z = ((PositionMsg*) t_msg)->z;
    }
    if(t_port->getID() == ports_id::IP_1) {
        _current_yaw = ((Vector3DMessage*) t_msg)->getData().x;
    }
}