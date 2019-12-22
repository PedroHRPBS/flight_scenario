#pragma once
#include "FlightElement.hpp"
#include "UpdatePoseReference.hpp"



UpdatePoseReference::UpdatePoseReference(){}

UpdatePoseReference::~UpdatePoseReference(){}

void UpdatePoseReference::perform(){
    
    //((MessageToBlock*)&_reference_update_msg)->target_block=this->target_block;
    this->emit_message((DataMessage*)&pose_reference);
}

void UpdatePoseReference::receive_msg_data(DataMessage* t_msg){}



