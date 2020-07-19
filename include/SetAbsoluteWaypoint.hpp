#pragma once
#include "common_srv/PosesMsg.hpp"
#include "FlightElement.hpp"
#include "PositionMsg.hpp"
#include "common_srv/Vector3DMessage.hpp"

class SetAbsoluteWaypoint : public FlightElement{

private:
    float _waypoint_x, _waypoint_y, _waypoint_z, _waypoint_yaw;
    float _current_x, _current_y, _current_z, _current_yaw;

public:
    void perform();
    void receiveMsgData(DataMessage* t_msg);
    SetAbsoluteWaypoint(float, float, float, float);
    ~SetAbsoluteWaypoint();
};