#pragma once
#include "FlightElement.hpp"
#include "UpdateReference.hpp"


void UpdateReference::perform(){
    ReferenceMessage _reference_update_msg;
    ((ControlMessage*)_reset_controller_msg)->target_block=this->target_block;
    this->emit_message((DataMessage*)&_reset_controller_msg);
}

void UpdateReference::receive_msg_data(DataMessage* t_msg){}

UpdateReference::UpdateReference(){}
UpdateReference::~UpdateReference(){}

