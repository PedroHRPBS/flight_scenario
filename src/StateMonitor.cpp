#include "StateMonitor.hpp"

StateMonitor::StateMonitor() {

}

StateMonitor::~StateMonitor() {

}

void StateMonitor::perform() {
    while(1){

        if(MainMissionStateManager.getMissionState() != uav_control_states::TAKING_OFF &&
            MainMissionStateManager.getMissionState() != uav_control_states::LANDING){

            if(_error > 0){
                MainMissionStateManager.updateMissionState(uav_control_states::ERROR);
            }else if(_number_of_waypoints > 0 && _armed){
                MainMissionStateManager.updateMissionState(uav_control_states::FOLLOWING_TRAJECTORY);
            }else if(_number_of_waypoints == 0 && _armed && _altitude > MIN_ALT_FOR_HOVERING){
                MainMissionStateManager.updateMissionState(uav_control_states::HOVERING);
            }else if(!_armed){
                MainMissionStateManager.updateMissionState(uav_control_states::LANDED);
            }

        }
        std::cout << "current state: " << (int)(MainMissionStateManager.getMissionState()) << std::endl;
        std::cout << "_armed: " << _armed << std::endl;
        std::cout << "_number_of_waypoints: " << _number_of_waypoints << std::endl;
        std::cout << "_altitude: " << _altitude << std::endl;
        std::cout << "_error: " << _error << std::endl;
        sleep(0.1);
    }
}

void StateMonitor::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::INFO){
        InfoMsg* info_msg = (InfoMsg*)t_msg;

        _number_of_waypoints = info_msg->number_of_waypoints;
        _armed = info_msg->armed;

    }else if(t_msg->getType() == msg_type::POSITION){
        PositionMsg* pos_msg = (PositionMsg*)t_msg;

        _altitude = pos_msg->z;

    }else if(t_msg->getType() == msg_type::ERROR){
        ErrorMsg* error_msg = (ErrorMsg*)t_msg;

        if(error_msg->error){
            _error++;
        }

    }
}