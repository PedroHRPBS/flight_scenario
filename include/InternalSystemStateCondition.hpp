#pragma once
#include "Condition.hpp"
#include "common_types.hpp"
#include "internal_states.hpp"
#include "MissionStateManager.hpp"

class InternalSystemStateCondition: public Condition {

private:
	bool _isConditionMet = false;
    uav_control_states m_check_state;

public:

    bool isConditionMet();

    void receive_msg_data(DataMessage* t_msg);

    InternalSystemStateCondition(uav_control_states);

};
