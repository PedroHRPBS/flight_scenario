#pragma once
#include "internal_states.hpp"
#include "FlightElement.hpp"
#include "IntegerMsg.hpp"
#include "MissionStateManager.hpp"

class ChangeInternalState : public FlightElement {

private:
    uav_control_states m_new_state;
    
public:
    void perform();
    void receiveMsgData(DataMessage*);

    ChangeInternalState(uav_control_states);
    ~ChangeInternalState();
};