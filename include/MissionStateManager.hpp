#pragma once
#include "MsgEmitter.hpp"
#include "MsgUavControlState.hpp"
#include "internal_states.hpp"
                                
class MissionStateManager : public msg_emitter
{
    private:
        uav_control_states current_uav_control_state = uav_control_states::WARMING_UP;
    public:
        void updateMissionState(uav_control_states);
        uav_control_states getMissionState();
};

extern MissionStateManager MainMissionStateManager;