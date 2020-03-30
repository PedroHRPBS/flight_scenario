#include "InternalSystemStateCondition.hpp"

InternalSystemStateCondition::InternalSystemStateCondition(uav_control_states t_check_state){
    m_check_state = t_check_state;
}

bool InternalSystemStateCondition::isConditionMet(){
	return (MainMissionStateManager.getMissionState() == m_check_state);
}

void InternalSystemStateCondition::receiveMsgData(DataMessage* t_msg){

}
    
