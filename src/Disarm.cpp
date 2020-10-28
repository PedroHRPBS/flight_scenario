
#include "Disarm.hpp"

// Disarm::Disarm(ros::NodeHandle& t_main_handler) : FlightElement(t_main_handler){

//     _disarm_client = t_main_handler.serviceClient<flight_controller::Arm>("arm");

// }
Disarm::Disarm(){
    _output_port_0 = new OutputPort(ports_id::OP_0, this);
    _ports = {_output_port_0};
}

Disarm::~Disarm(){
    
}

void Disarm::perform(){
    
    ArmDataMessage _arm_message;
    _arm_message.isArmed = false;
    this->_output_port_0->receiveMsgData((DataMessage*)&_arm_message);
}