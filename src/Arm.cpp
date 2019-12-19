#include "Arm.hpp"

Arm::Arm() {

}

Arm::~Arm() {

}

void Arm::perform()
{
    ArmDataMessage _arm_message;
    _arm_message.isArmed=1;
    this->emit_message((DataMessage*)&_arm_message);
}

void Arm::receive_msg_data(DataMessage* t_msg){

}