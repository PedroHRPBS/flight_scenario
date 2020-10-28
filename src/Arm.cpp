#include "Arm.hpp"

// Arm::Arm(ros::NodeHandle& t_main_handler) : FlightElement(t_main_handler) {

//     _arm_client = t_main_handler.serviceClient<flight_controller::Arm>("arm");

// }

Arm::Arm() {
    _output_port_0 = new OutputPort(ports_id::OP_0, this);
    _ports = {_output_port_0};
}

Arm::~Arm() {

}

void Arm::perform()
{
    ArmDataMessage _arm_message;
    _arm_message.isArmed = true;
    this->_output_port_0->receiveMsgData((DataMessage*)&_arm_message);
}