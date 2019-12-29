#include "ROSUnit_UpdateController.hpp"

ROSUnit_UpdateController::ROSUnit_UpdateController(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler){

    _update_controller_client = t_main_handler.serviceClient<positioning_system::Update_Controller>("update_controller");

}

ROSUnit_UpdateController::~ROSUnit_UpdateController() {

}

void ROSUnit_UpdateController::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::pid_data_update){

        PIDDataMessage* _update_msg = (PIDDataMessage*)t_msg;
        
        positioning_system::Update_Controller srv;

        srv.request.controller_parameters.pid_kp = _update_msg->PIDdata.kp;
        srv.request.controller_parameters.pid_ki = _update_msg->PIDdata.ki;
        srv.request.controller_parameters.pid_kd = _update_msg->PIDdata.kd;
        srv.request.controller_parameters.pid_kdd = _update_msg->PIDdata.kdd;
        srv.request.controller_parameters.pid_anti_windup = _update_msg->PIDdata.anti_windup;
        srv.request.controller_parameters.pid_en_pv_derivation = _update_msg->PIDdata.en_pv_derivation;
        srv.request.controller_parameters.id = (int)(_update_msg->PIDdata.id);

        bool success = _update_controller_client.call(srv);

        if (success)
        {
            ROS_INFO("CONTROLLER UPDATED");
        }
        else 
        {
            ROS_ERROR("Failed to call service /update_controller");
        }

    }
}