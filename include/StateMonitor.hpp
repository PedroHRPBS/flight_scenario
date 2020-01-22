#pragma once
#include "FlightElement.hpp"
#include "InfoMsg.hpp"
#include "PositionMsg.hpp"
#include "ErrorMsg.hpp"
#include "IntegerMsg.hpp"
#include "MissionStateManager.hpp"
#include "internal_states.hpp"
const float MIN_ALT_FOR_HOVERING = 0.2;


class StateMonitor : public FlightElement{

private:
    int _number_of_waypoints, _error = 0;
    bool _armed;
    float _altitude;
    
public:

    void perform();
    void receive_msg_data(DataMessage*);

    StateMonitor();
    ~StateMonitor();
};