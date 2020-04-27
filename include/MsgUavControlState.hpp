#pragma once
#include "common_srv/DataMessage.hpp"
#include "internal_states.hpp"

class MsgUavControlState : public DataMessage{
public:
    uav_control_states msg_uav_control_state;
    msg_type getType();
    const int getSize();
};