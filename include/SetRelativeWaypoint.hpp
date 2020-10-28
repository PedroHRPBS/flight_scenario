#pragma once
#include "common_srv/PosesMsg.hpp"
#include "FlightElement.hpp"
#include "PositionMsg.hpp"
#include "common_srv/Vector3DMessage.hpp"

class SetRelativeWaypoint : public FlightElement{

private:
    Port* _input_port_0;
    Port* _input_port_1;
    Port* _output_port_0;
    float _waypoint_x, _waypoint_y, _waypoint_z, _waypoint_yaw;
    float _current_x, _current_y, _current_z, _current_yaw;

public:
    enum ports_id {IP_0, IP_1, OP_0};
    void process(DataMessage* t_msg, Port* t_port);
    void perform();
    SetRelativeWaypoint(float, float, float, float);
    ~SetRelativeWaypoint();
};