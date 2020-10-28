#include "ROSUnit_Arm.hpp"

ROSUnit_Arm::ROSUnit_Arm(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _input_port_0 = new InputPort(ports_id::IP_0, this);
    _ports = {_input_port_0};
    _arm_client = t_main_handler.serviceClient<flight_controller::Arm>("arm");
}

ROSUnit_Arm::~ROSUnit_Arm() {

}

void ROSUnit_Arm::process(DataMessage* t_msg, Port* t_port) {
    if(t_port->getID() == ports_id::IP_0)
    {
        ArmDataMessage* _arm_msg = (ArmDataMessage*)t_msg;
        flight_controller::Arm srv;
        srv.request.armed = _arm_msg->isArmed;
        bool success = _arm_client.call(srv);

        if (success && _arm_msg->isArmed)
        {
            ROS_INFO("ARMED");
        }
        else if(success && !_arm_msg->isArmed){
            ROS_INFO("DISARMED");
        }
        else 
        {
            ROS_ERROR("Failed to call service /arm");
        }
    }
}