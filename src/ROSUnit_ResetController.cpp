#include "ROSUnit_ResetController.hpp"

ROSUnit_ResetController::ROSUnit_ResetController(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler){
    _input_port_0 = new InputPort(ports_id::IP_0, this);
    _ports = {_input_port_0};
    _reset_controller_client = t_main_handler.serviceClient<flight_controller::Reset_Controller>("reset_controller");
}

ROSUnit_ResetController::~ROSUnit_ResetController() {

}

void ROSUnit_ResetController::process(DataMessage* t_msg, Port* t_port) {
    if(t_port->getID() == ports_id::IP_0) {

        ResetControllerMessage* _reset_msg = (ResetControllerMessage*)t_msg;
        
        flight_controller::Reset_Controller srv;

        MessageToBlock* _msg_block = (MessageToBlock*)_reset_msg;

        block_id id = _msg_block->target_block;

        srv.request.id = (int)id;

        bool success = _reset_controller_client.call(srv);

        if (success)
        {
            ROS_INFO("CONTROLLER RESET");
        }
        else 
        {
            ROS_ERROR("Failed to call service /update_controller");
        }
    }
}