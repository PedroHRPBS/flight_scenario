#include "MissionStateManager.hpp"
MissionStateManager MainMissionStateManager;

void MissionStateManager::updateMissionState(uav_control_states t_current_state){
    current_uav_control_state = t_current_state;
    MsgUavControlState state_msg;
    state_msg.msg_uav_control_state=t_current_state;
    emitMsgUnicastDefault(&state_msg);
}

uav_control_states MissionStateManager::getMissionState(){
    return current_uav_control_state;
}
