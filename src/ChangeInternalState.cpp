#include "ChangeInternalState.hpp"

ChangeInternalState::ChangeInternalState(uav_control_states t_new_state) {
    m_new_state = t_new_state;
}

ChangeInternalState::~ChangeInternalState() {

}

void ChangeInternalState::perform() {
    MainMissionStateManager.updateMissionState(m_new_state);
    std::cout << "Current state: " << (int)m_new_state << std::endl;
}

void ChangeInternalState::receive_msg_data(DataMessage* t_msg){
    
    if(t_msg->getType() == msg_type::INTEGER){
        IntegerMsg* int_msg = (IntegerMsg*)t_msg;

        MainMissionStateManager.updateMissionState(static_cast<uav_control_states>(m_new_state));
        
    }
}