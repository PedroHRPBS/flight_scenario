
#include "Unarm.hpp"



void Unarm::perform(){
    ArmDataMessage _arm_message;
    _arm_message.isArmed=0;
    this->emit_message((DataMessage*)&_arm_message);
}
void Unarm::receive_msg_data(DataMessage* t_msg){

}

Unarm::Unarm(ros::NodeHandle& t_main_handler) : FlightElement(t_main_handler){

}
Unarm::~Unarm(){
    
}
