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

        srv.request.parameters.kp = _update_msg->PIDdata.kp;
        srv.request.parameters.ki = _update_msg->PIDdata.ki;
        srv.request.parameters.kd = _update_msg->PIDdata.kd;
        srv.request.parameters.kdd = _update_msg->PIDdata.kdd;
        srv.request.parameters.anti_windup = _update_msg->PIDdata.anti_windup;
        srv.request.parameters.en_pv_derivation = _update_msg->PIDdata.en_pv_derivation;
        srv.request.parameters.id = (int)(_update_msg->PIDdata.id);

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