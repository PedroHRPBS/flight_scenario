#pragma once
#include "FlightElement.hpp"
#include "InfoMsg.hpp"
#include "PositionMsg.hpp"
#include "ErrorMsg.hpp"
#include "common_srv/IntegerMsg.hpp"
#include "MissionStateManager.hpp"
#include "internal_states.hpp"

const float MIN_ALT_FOR_HOVERING = 0.2;


class StateMonitor : public FlightElement{

private:
    int _number_of_waypoints, _error = 0;
    bool _armed;
    float _altitude;
    IntegerMsg int_msg;
    uav_control_states old_state = uav_control_states::HOVERING;
public:

    void perform();
    //void receiveMsgData(DataMessage*);

    StateMonitor();
    ~StateMonitor();
};