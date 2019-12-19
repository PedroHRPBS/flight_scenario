#include "Arm.hpp"

Arm::Arm(ros::NodeHandle& t_main_handler) : FlightElement(t_main_handler) {

    arm_client = t_main_handler.serviceClient<

}

Arm::~Arm() {

}

void Arm::perform()
{
    // ArmDataMessage _arm_message;
    // _arm_message.isArmed = 1;
    // this->emit_message((DataMessage*)&_arm_message);

}

void Arm::receive_msg_data(DataMessage* t_msg){

}