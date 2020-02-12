#pragma once
#include "PosesMsg.hpp"
#include "FlightElement.hpp"
#include "PositionMsg.hpp"
#include "Vector3DMessage.hpp"

class SetRelativeWaypoint : public FlightElement{

private:
    float _waypoint_x, _waypoint_y, _waypoint_z, _waypoint_yaw;
    float _current_x, _current_y, _current_z, _current_yaw;

public:
    void perform();
    void receive_msg_data(DataMessage* t_msg);
    SetRelativeWaypoint(float, float, float, float);
    ~SetRelativeWaypoint();
};