#include "StateMonitor.hpp"

StateMonitor::StateMonitor() {

}

StateMonitor::~StateMonitor() {

}

void StateMonitor::perform() {
    while(1){
        //TODO remove TAKING_OFF, considering it's being done by Outdoor navigation
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

        if(MainMissionStateManager.getMissionState() != old_state){
            int_msg.data = (int)(MainMissionStateManager.getMissionState());
            // TODO: fiix
            //this->emitMsgUnicastDefault((DataMessage*) &int_msg);
            old_state = MainMissionStateManager.getMissionState();
        }
        

        sleep(0.1);
    }
}

// void StateMonitor::receiveMsgData(DataMessage* t_msg){
//     TODO: fix
//     if(t_msg->getType() == msg_type::INFO){
//         InfoMsg* info_msg = (InfoMsg*)t_msg;

//         _number_of_waypoints = info_msg->number_of_waypoints;
//         _armed = info_msg->armed;

//     }else if(t_msg->getType() == msg_type::POSITION){
//         PositionMsg* pos_msg = (PositionMsg*)t_msg;

//         _altitude = pos_msg->z;

//     }else if(t_msg->getType() == msg_type::ERROR){
//         ErrorMsg* error_msg = (ErrorMsg*)t_msg;

//         if(error_msg->error){
//             _error++;
//         }

//     }
// }