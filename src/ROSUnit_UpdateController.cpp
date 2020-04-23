#include "ROSUnit_UpdateController.hpp"

ROSUnit_UpdateController::ROSUnit_UpdateController(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler){

    _update_controller_pid_client = t_main_handler.serviceClient<flight_controller::Update_Controller_PID>("update_controller/pid");
    _update_controller_mrft_client = t_main_handler.serviceClient<flight_controller::Update_Controller_MRFT>("update_controller/mrft");
    _update_controller_sm_client = t_main_handler.serviceClient<flight_controller::Update_Controller_SM>("update_controller/sm");

}

ROSUnit_UpdateController::~ROSUnit_UpdateController() {

}

void ROSUnit_UpdateController::receiveMsgData(DataMessage* t_msg){

}

void ROSUnit_UpdateController::receiveMsgData(DataMessage* t_msg, int t_channel){

    if(t_msg->getType() == msg_type::UPDATECONTROLLER){

        if(t_channel == receiving_channels::pid){

            ControllerMessage* _update_msg = (ControllerMessage*)t_msg;
        
            flight_controller::Update_Controller_PID srv;
            srv.request.controller_parameters.id = (int)(_update_msg->getID());
            srv.request.controller_parameters.pid_kp = _update_msg->getPIDParam().kp;
            srv.request.controller_parameters.pid_ki = _update_msg->getPIDParam().ki;
            srv.request.controller_parameters.pid_kd = _update_msg->getPIDParam().kd;
            srv.request.controller_parameters.pid_kdd = _update_msg->getPIDParam().kdd;
            srv.request.controller_parameters.pid_anti_windup = _update_msg->getPIDParam().anti_windup;
            srv.request.controller_parameters.pid_en_pv_derivation = _update_msg->getPIDParam().en_pv_derivation;
            bool success = _update_controller_pid_client.call(srv);
            if (success)
            {
                ROS_INFO("CONTROLLER UPDATED. id: %d", srv.request.controller_parameters.id);
            }
            else 
            {
                ROS_ERROR("Failed to call service /update_controller");
            }

        }else if(t_channel == receiving_channels::mrft){

            ControllerMessage* _update_msg = (ControllerMessage*)t_msg;
        
            flight_controller::Update_Controller_MRFT srv;
            srv.request.controller_parameters.id = (int)(_update_msg->getID());
            srv.request.controller_parameters.mrft_beta = _update_msg->getMRFTParam().beta;
            srv.request.controller_parameters.mrft_relay_amp = _update_msg->getMRFTParam().relay_amp;
            srv.request.controller_parameters.mrft_bias = _update_msg->getMRFTParam().bias;

            bool success = _update_controller_mrft_client.call(srv);
            if (success)
            {
                ROS_INFO("CONTROLLER UPDATED. id: %d", srv.request.controller_parameters.id);
            }
            else 
            {
                ROS_ERROR("Failed to call service /update_controller");
            }


        }else if(t_channel == receiving_channels::sm){
            ControllerMessage* _update_msg = (ControllerMessage*)t_msg;
        
            flight_controller::Update_Controller_SM srv;
            srv.request.controller_parameters.id = (int)(_update_msg->getID());
            srv.request.controller_parameters.sm_alpha1 = _update_msg->getSMParam().alpha1;
            srv.request.controller_parameters.sm_alpha2 = _update_msg->getSMParam().alpha2;
            srv.request.controller_parameters.sm_h1 = _update_msg->getSMParam().h1;
            srv.request.controller_parameters.sm_h2 = _update_msg->getSMParam().h2;
            bool success = _update_controller_sm_client.call(srv);
            if (success)
            {
                ROS_INFO("CONTROLLER UPDATED. id: %d", srv.request.controller_parameters.id);
            }
            else 
            {
                ROS_ERROR("Failed to call service /update_controller");
            }
        }
    }

}