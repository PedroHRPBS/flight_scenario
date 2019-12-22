#pragma once
#include "FlightElement.hpp"
#include "UpdateReference.hpp"



UpdateReference::UpdateReference(){}

UpdateReference::~UpdateReference(){}

void UpdateReference::perform(){
    ReferenceMessage _reference_update_msg;
    ((MessageToBlock*)&_reference_update_msg)->target_block=this->target_block;
    this->emit_message((DataMessage*)&_reference_update_msg);
}

void UpdateReference::receive_msg_data(DataMessage* t_msg){}



