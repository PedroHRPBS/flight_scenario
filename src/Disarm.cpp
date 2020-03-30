
#include "Disarm.hpp"

// Disarm::Disarm(ros::NodeHandle& t_main_handler) : FlightElement(t_main_handler){

//     _disarm_client = t_main_handler.serviceClient<flight_controller::Arm>("arm");

// }
Disarm::Disarm(){
    
}

Disarm::~Disarm(){
    
}

void Disarm::perform(){
    
    ArmDataMessage _arm_message;
    _arm_message.isArmed = false;
    this->emitMsgUnicastDefault((DataMessage*)&_arm_message);

}
void Disarm::receiveMsgData(DataMessage* t_msg){

}


